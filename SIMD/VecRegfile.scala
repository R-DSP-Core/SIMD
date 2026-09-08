package zircon.simd

import chisel3._
import chisel3.util._

/** 目前提供加法、乘法与外积乘加三种运算编码。 */
object SimdOp {
    val ADD   = 0.U(2.W)
    val MUL   = 1.U(2.W)
    val OMAC  = 2.U(2.W)
    val width = 2
    val lanes = 8
    val nBank = 8
}

class VecRegfileIO extends Bundle {
    val rbank    = Input(UInt(log2Ceil(SimdOp.nBank).W))
    val wbank    = Input(UInt(log2Ceil(SimdOp.nBank).W))
    val rdata    = Output(Vec(SimdOp.lanes, UInt(32.W)))
    val wen      = Input(Vec(SimdOp.lanes, Bool()))
    val wdata    = Input(Vec(SimdOp.lanes, UInt(32.W)))
    val clearAll = Input(Bool())
}

/** 八组、每组八路、每路三十二位的单份向量累加寄存器堆。同组写入在当拍对读端口可见。 */
class VecRegfile extends Module {
    val io = IO(new VecRegfileIO)

    val banks = RegInit(
        VecInit.tabulate(SimdOp.nBank)(_ =>
            VecInit.tabulate(SimdOp.lanes)(_ => 0.U(32.W))))

    when(io.clearAll) {
        for (b <- 0 until SimdOp.nBank; l <- 0 until SimdOp.lanes) {
            banks(b)(l) := 0.U
        }
    }.otherwise {
        for (l <- 0 until SimdOp.lanes) {
            when(io.wen(l)) {
                banks(io.wbank)(l) := io.wdata(l)
            }
        }
    }

    for (l <- 0 until SimdOp.lanes) {
        val sameBankFwd =
            io.wen(l) && !io.clearAll && (io.rbank === io.wbank)
        io.rdata(l) := Mux(sameBankFwd, io.wdata(l),
            Mux1H((0 until SimdOp.nBank).map(b =>
                (io.rbank === b.U) -> banks(b)(l))))
    }
}
