package zircon.simd

import chisel3._
import chisel3.util._

class SimdAddPort extends Bundle {
    val valid  = Input(Bool())
    val stall  = Output(Bool())
    val srcA   = Input(Vec(SimdOp.lanes, UInt(32.W)))
    val srcB   = Input(Vec(SimdOp.lanes, UInt(32.W)))
    val result = Output(Vec(SimdOp.lanes, UInt(32.W)))
    val done   = Output(Bool())
}

class SimdMulPort extends Bundle {
    val valid     = Input(Bool())
    val stall     = Output(Bool())
    val op        = Input(UInt(SimdOp.width.W))
    val bank      = Input(UInt(log2Ceil(SimdOp.nBank).W))
    val dstToVReg = Input(Bool())
    val srcA      = Input(Vec(SimdOp.lanes, UInt(32.W)))
    val srcB      = Input(Vec(SimdOp.lanes, UInt(32.W)))
    val result    = Output(Vec(SimdOp.lanes, UInt(32.W)))
    val writeVreg = Output(Bool())
    val done      = Output(Bool())
    val doneBank  = Output(UInt(log2Ceil(SimdOp.nBank).W))
}

/** 供外部读取或整堆清零。搬移与清空不作为本核操作码。 */
class SimdVregPort extends Bundle {
    val rbank      = Input(UInt(log2Ceil(SimdOp.nBank).W))
    val rdata      = Output(Vec(SimdOp.lanes, UInt(32.W)))
    val clearAll   = Input(Bool())
    val readingBusy = Output(Bool())
}

/** 向量纯计算核：加法口与乘法口各自给出完成信号，内部不合并写回总线。冲刷只作废尚未写回的在途操作 */
class SimdCompute extends Module {
    val io = IO(new Bundle {
        val flush = Input(Bool())
        val add   = new SimdAddPort
        val mul   = new SimdMulPort
        val vreg  = new SimdVregPort
    })

    val vreg = Module(new VecRegfile)
    val add  = Module(new SimdAddPipe)
    val mul  = Module(new SimdMulPipe)

    add.io.flush := io.flush
    mul.io.flush := io.flush

    add.io.valid := io.add.valid
    add.io.srcA  := io.add.srcA
    add.io.srcB  := io.add.srcB
    io.add.stall  := false.B
    io.add.result := add.io.result
    io.add.done   := add.io.done

    mul.io.valid     := io.mul.valid
    mul.io.op        := io.mul.op
    mul.io.bank      := io.mul.bank
    mul.io.dstToVReg := io.mul.dstToVReg
    mul.io.srcA      := io.mul.srcA
    mul.io.srcB      := io.mul.srcB
    io.mul.stall     := mul.io.stall
    io.mul.result    := mul.io.result
    io.mul.writeVreg := mul.io.writeVreg
    io.mul.done      := mul.io.done
    io.mul.doneBank  := mul.io.doneBank

    // 外积乘加处于第四执行级时占用读端口，其余周期把读端口交给外部。
    io.vreg.readingBusy := mul.io.vregReading
    vreg.io.rbank := Mux(mul.io.vregReading, mul.io.vecReg.rbank, io.vreg.rbank)
    mul.io.vecReg.rdata := vreg.io.rdata
    io.vreg.rdata       := vreg.io.rdata

    vreg.io.wbank    := mul.io.vecReg.wbank
    vreg.io.wen      := mul.io.vecReg.wen
    vreg.io.wdata    := mul.io.vecReg.wdata
    vreg.io.clearAll := io.vreg.clearAll
}

/** 旧名保留，新代码请例化 SimdCompute。 */
class SimdFu extends SimdCompute
