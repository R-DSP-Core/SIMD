package zircon.simd

import chisel3._
import chisel3.util._

class SimdPipeCtrl extends Bundle {
    val valid     = Bool()
    val op        = UInt(SimdOp.width.W)
    val bank      = UInt(log2Ceil(SimdOp.nBank).W)
    val dstToVReg = Bool()
}

class SimdAddPipeIO extends Bundle {
    val valid  = Input(Bool())
    val flush  = Input(Bool())
    val srcA   = Input(Vec(SimdOp.lanes, UInt(32.W)))
    val srcB   = Input(Vec(SimdOp.lanes, UInt(32.W)))
    val result = Output(Vec(SimdOp.lanes, UInt(32.W)))
    val done   = Output(Bool())
}

/** 八路并行加法流水线：打拍输入后组合相加，再打拍写出。 */
class SimdAddPipe extends Module {
    val io = IO(new SimdAddPipeIO)
    private val adders = VecInit.tabulate(SimdOp.lanes)(_ => Module(new SimdPAdder32).io)

    val validEX = RegNext(io.valid && !io.flush, false.B)
    val srcAEX  = RegNext(io.srcA, VecInit.fill(SimdOp.lanes)(0.U(32.W)))
    val srcBEX  = RegNext(io.srcB, VecInit.fill(SimdOp.lanes)(0.U(32.W)))

    val aluRes = Wire(Vec(SimdOp.lanes, UInt(32.W)))
    for (l <- 0 until SimdOp.lanes) {
        adders(l).src1 := srcAEX(l)
        adders(l).src2 := srcBEX(l)
        adders(l).cin  := 0.U
        aluRes(l)      := adders(l).res
    }

    val validWB = RegNext(validEX && !io.flush, false.B)
    io.result := RegNext(aluRes, VecInit.fill(SimdOp.lanes)(0.U(32.W)))
    io.done   := validWB
}

class SimdMulVregIO extends Bundle {
    val rbank = Output(UInt(log2Ceil(SimdOp.nBank).W))
    val rdata = Input(Vec(SimdOp.lanes, UInt(32.W)))
    val wbank = Output(UInt(log2Ceil(SimdOp.nBank).W))
    val wen   = Output(Vec(SimdOp.lanes, Bool()))
    val wdata = Output(Vec(SimdOp.lanes, UInt(32.W)))
}

class SimdMulPipeIO extends Bundle {
    val valid       = Input(Bool())
    val stall       = Output(Bool())
    val flush       = Input(Bool())
    val op          = Input(UInt(SimdOp.width.W))
    val bank        = Input(UInt(log2Ceil(SimdOp.nBank).W))
    val dstToVReg   = Input(Bool())
    val srcA        = Input(Vec(SimdOp.lanes, UInt(32.W)))
    val srcB        = Input(Vec(SimdOp.lanes, UInt(32.W)))
    val vecReg      = new SimdMulVregIO
    val vregReading = Output(Bool())
    val result      = Output(Vec(SimdOp.lanes, UInt(32.W)))
    val writeVreg   = Output(Bool())
    val done        = Output(Bool())
    val doneBank    = Output(UInt(log2Ceil(SimdOp.nBank).W))
}

/**
 * 乘法沿读数、三级执行与写回流出；外积乘加在写回前再插入一级累加。
 * 紧随外积乘加的乘法会被反压一拍，以免两路在同一拍争用写口。
 */
class SimdMulPipe extends Module {
    val io = IO(new SimdMulPipeIO)
    private val macAdders = VecInit.tabulate(SimdOp.lanes)(_ => Module(new SimdPAdder32).io)
    private val muls      = VecInit.tabulate(SimdOp.lanes)(_ => Module(new SimdMul32).io)
    private def bubble    = 0.U.asTypeOf(new SimdPipeCtrl)

    def isMac(op: UInt): Bool = op === SimdOp.OMAC
    def isMul(op: UInt): Bool = op === SimdOp.MUL

    val lastWasMac = RegInit(false.B)
    val mulAfterMac = io.valid && isMul(io.op) && lastWasMac
    io.stall := mulAfterMac
    val accept = io.valid && !io.stall

    when(io.flush) {
        lastWasMac := false.B
    }.elsewhen(accept) {
        lastWasMac := isMac(io.op)
    }.otherwise {
        lastWasMac := false.B
    }

    val ctrlIn = Wire(new SimdPipeCtrl)
    ctrlIn.valid     := accept
    ctrlIn.op        := io.op
    ctrlIn.bank      := io.bank
    ctrlIn.dstToVReg := io.dstToVReg

    val ctrlRF = RegNext(Mux(io.flush || !accept, bubble, ctrlIn), bubble)
    val srcARF = RegNext(io.srcA, VecInit.fill(SimdOp.lanes)(0.U(32.W)))
    val srcBRF = RegNext(io.srcB, VecInit.fill(SimdOp.lanes)(0.U(32.W)))

    val isOmacRF = ctrlRF.valid && isMac(ctrlRF.op)
    val vsrc1RF = Wire(Vec(SimdOp.lanes, UInt(32.W)))
    val vsrc2RF = Wire(Vec(SimdOp.lanes, UInt(32.W)))
    for (l <- 0 until SimdOp.lanes) {
        vsrc1RF(l) := Mux(isOmacRF, srcARF(0), srcARF(l))
        vsrc2RF(l) := srcBRF(l)
    }

    val ctrlEX1 = RegNext(Mux(io.flush, bubble, ctrlRF), bubble)
    val vsrc1EX1 = RegNext(vsrc1RF, VecInit.fill(SimdOp.lanes)(0.U(32.W)))
    val vsrc2EX1 = RegNext(vsrc2RF, VecInit.fill(SimdOp.lanes)(0.U(32.W)))

    for (l <- 0 until SimdOp.lanes) {
        muls(l).src1 := vsrc1EX1(l)
        muls(l).src2 := vsrc2EX1(l)
    }

    val ctrlEX2 = RegNext(Mux(io.flush, bubble, ctrlEX1), bubble)
    val ctrlEX3 = RegNext(Mux(io.flush, bubble, ctrlEX2), bubble)

    val mulResEX3 = Wire(Vec(SimdOp.lanes, UInt(32.W)))
    for (l <- 0 until SimdOp.lanes) {
        mulResEX3(l) := muls(l).res
    }

    val isMacEX3 = ctrlEX3.valid && isMac(ctrlEX3.op)

    val ctrlWB_mul = RegNext(Mux(io.flush || isMacEX3, bubble, ctrlEX3), bubble)
    val mulResWB   = RegNext(mulResEX3, VecInit.fill(SimdOp.lanes)(0.U(32.W)))

    val ctrlEX4  = RegNext(Mux(io.flush || !isMacEX3, bubble, ctrlEX3), bubble)
    val mulResEX4 = RegNext(mulResEX3, VecInit.fill(SimdOp.lanes)(0.U(32.W)))

    // 对同一寄存器组连续发射的外积乘加，用旁路提供尚未写入的累加中间结果。
    val macAccFwd     = RegInit(VecInit.fill(SimdOp.lanes)(0.U(32.W)))
    val macAccFwdVld  = RegInit(false.B)
    val macAccFwdBank = RegInit(0.U(log2Ceil(SimdOp.nBank).W))
    val ex4Bank = ctrlEX4.bank

    io.vregReading  := ctrlEX4.valid
    io.vecReg.rbank := ex4Bank

    val macResEX4 = Wire(Vec(SimdOp.lanes, UInt(32.W)))
    for (l <- 0 until SimdOp.lanes) {
        val fwdOk = macAccFwdVld && (macAccFwdBank === ex4Bank)
        val accIn = Mux(fwdOk, macAccFwd(l), io.vecReg.rdata(l))
        macAdders(l).src1 := mulResEX4(l)
        macAdders(l).src2 := accIn
        macAdders(l).cin  := 0.U
        macResEX4(l)      := macAdders(l).res
    }

    when(io.flush) {
        macAccFwdVld := false.B
    }.elsewhen(ctrlEX4.valid) {
        macAccFwd     := macResEX4
        macAccFwdVld  := true.B
        macAccFwdBank := ex4Bank
    }.otherwise {
        macAccFwdVld := false.B
    }

    val ctrlWB_mac = RegNext(Mux(io.flush || !ctrlEX4.valid, bubble, ctrlEX4), bubble)
    val macResWB   = RegNext(macResEX4, VecInit.fill(SimdOp.lanes)(0.U(32.W)))

    val mulWb = ctrlWB_mul.valid
    val macWb = ctrlWB_mac.valid
    assert(!(mulWb && macWb), "乘法与外积乘加不得在同一拍写回")

    val macWbBank = ctrlWB_mac.bank
    io.vecReg.wbank := Mux(macWb, macWbBank, 0.U)

    for (l <- 0 until SimdOp.lanes) {
        io.result(l) := Mux(macWb, macResWB(l), mulResWB(l))
        io.vecReg.wen(l) := macWb || (mulWb && ctrlWB_mul.dstToVReg)
        io.vecReg.wdata(l) := Mux(macWb, macResWB(l), mulResWB(l))
    }

    io.writeVreg := macWb || (mulWb && ctrlWB_mul.dstToVReg)
    io.done      := mulWb || macWb
    io.doneBank  := Mux(macWb, macWbBank, 0.U)
}
