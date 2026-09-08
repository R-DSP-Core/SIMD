package zircon.simd.packsimd

import chisel3._
import chisel3.util._

/** SIMD 加法器端口。SIMDMode: 00/11=1x32, 01=4x8, 10=2x16。 */
class AdderIO(n: Int) extends Bundle {
    val src1     = Input(UInt(n.W))
    val src2     = Input(UInt(n.W))
    val SIMDMode = Input(UInt(2.W))
    val cin      = Input(UInt(1.W))
    val sat      = Input(Bool()) // 饱和运算使能
    val res      = Output(UInt(n.W))
    val cout     = Output(UInt(1.W))
}

/** 4-bit Brent-Kung 风格进位：输入 p/g/cin，输出组传播、组生成、组内进位。 */
object BLevelCarry4 {
    def apply(p: UInt, g: UInt, c: UInt): (UInt, UInt, UInt) = {
        assert(p.getWidth == 4, "p must be 4 bits wide")
        assert(g.getWidth == 4, "g must be 4 bits wide")
        assert(c.getWidth == 1, "c must be 1 bits wide")
        val pn = p.andR
        val gn = g(3) | (p(3) & g(2)) | (p(3) & p(2) & g(1)) | (p(3) & p(2) & p(1) & g(0))
        val cn = Wire(Vec(4, UInt(1.W)))
        cn(0) := g(0) | (p(0) & c)
        cn(1) := g(1) | (p(1) & g(0)) | (p(1) & p(0) & c)
        cn(2) := g(2) | (p(2) & g(1)) | (p(2) & p(1) & g(0)) | (p(2) & p(1) & p(0) & c)
        cn(3) := gn | (pn & c)
        (pn, gn, cn.asUInt)
    }
}

/** 有符号饱和：正溢出钳到 max，负溢出钳到 min；按 SIMDMode 分 lane 处理。 */
object SatSIMD {
    def apply(src1: UInt, src2: UInt, res: UInt, SIMDMode: UInt): UInt = {
        val satRes = WireDefault(res)
        switch(SIMDMode) {
            is("b00".U, "b11".U) { // 32
                val signPos = ~src1(31) & ~src2(31) & res(31)  // 正溢出
                val signNeg = src1(31) & src2(31) & ~res(31)   // 负溢出
                satRes := Mux(signPos === 1.U, "h7FFFFFFF".U, Mux(signNeg === 1.U, "h80000000".U, res))
            }
            is("b01".U) { // 4x8
                val lanes = Wire(Vec(4, UInt(8.W)))
                for (i <- 0 until 4) {
                    val signPos = ~src1(i * 8 + 7) & ~src2(i * 8 + 7) & res(i * 8 + 7)
                    val signNeg = src1(i * 8 + 7) & src2(i * 8 + 7) & ~res(i * 8 + 7)
                    lanes(i) := Mux(signPos === 1.U, "h7F".U, Mux(signNeg === 1.U, "h80".U, res(i * 8 + 7, i * 8)))
                }
                satRes := Cat(lanes.reverse)
            }
            is("b10".U) { // 2x16
                val lanes = Wire(Vec(2, UInt(16.W)))
                for (i <- 0 until 2) {
                    val signPos = ~src1(i * 16 + 15) & ~src2(i * 16 + 15) & res(i * 16 + 15)
                    val signNeg = src1(i * 16 + 15) & src2(i * 16 + 15) & ~res(i * 16 + 15)
                    lanes(i) := Mux(signPos === 1.U, "h7FFF".U, Mux(signNeg === 1.U, "h8000".U, res(i * 16 + 15, i * 16)))
                }
                satRes := Cat(lanes.reverse)
            }
        }
        satRes
    }
}

/** 32-bit packed-SIMD 并行前缀加法器：在 lane 边界切断进位，可选饱和。 */
class BLevelPAdder32_SIMD extends Module {
    val io = IO(new AdderIO(32))
    val pi = io.src1 | io.src2
    val gi = io.src1 & io.src2

    val p = Wire(MixedVec(Vec(8, UInt(1.W)), Vec(2, UInt(1.W))))
    val g = Wire(MixedVec(Vec(8, UInt(1.W)), Vec(2, UInt(1.W))))
    val c = Wire(MixedVec(Vec(8, UInt(4.W)), Vec(2, UInt(4.W)), Vec(1, UInt(4.W))))

    // 第一层：8 个 4-bit 组。lane 起点注入 io.cin（ADD=0 / SUB=1），避免跨 lane 借用 16-bit 前缀。
    for (i <- 0 until 8) {
        val cin = WireDefault(0.U(1.W))
        if (i == 0) cin := io.cin
        else {
            switch(io.SIMDMode) {
                is("b00".U, "b11".U) {
                    if (i == 4) cin := c(2).asUInt(0)
                    else cin := c(1).asUInt(i - 1)
                }
                is("b01".U) { // 4x8：偶数组是新 lane；奇数组只用本 lane 内上一 4-bit 组的 cout
                    if ((i % 2) == 0) cin := io.cin
                    else cin := c(0)(i - 1)(3).asUInt
                }
                is("b10".U) { // 2x16：bit16 起新 lane
                    if (i == 4) cin := io.cin
                    else cin := c(1).asUInt(i - 1)
                }
            }
        }
        val (p0n, g0n, c0n) = BLevelCarry4(pi(i * 4 + 3, i * 4), gi(i * 4 + 3, i * 4), cin)
        p(0)(i) := p0n
        g(0)(i) := g0n
        c(0)(i) := c0n
    }

    // 第二层：2 个 16-bit 组。非 32-bit 时高半字 cin 用 io.cin，切断跨半字进位。
    for (i <- 0 until 2) {
        val cin = WireDefault(0.U(1.W))
        if (i == 0) cin := io.cin
        else {
            cin := Mux(io.SIMDMode === "b00".U || io.SIMDMode === "b11".U, c(2).asUInt(i - 1), io.cin)
        }
        val (p1n, g1n, c1n) = BLevelCarry4(p(0).asUInt(i * 4 + 3, i * 4), g(0).asUInt(i * 4 + 3, i * 4), cin)
        p(1)(i) := p1n
        g(1)(i) := g1n
        c(1)(i) := c1n
    }

    // 第三层：顶层 32-bit 组进位
    val (p2n, g2n, c2n) = BLevelCarry4(0.U(2.W) ## p(1).asUInt, 0.U(2.W) ## g(1).asUInt, io.cin)
    c(2)(0) := c2n

    // 求和用进位：先切断跨 lane 的 cout，再把 io.cin 填回各 lane 起点（供 packed SUB）
    val carryBase = (c(0).asUInt(30, 0) ## io.cin)
    val finalC    = Wire(UInt(32.W))
    when(io.SIMDMode === "b01".U) {
        val laneCin = Cat(0.U(7.W), io.cin, 0.U(7.W), io.cin, 0.U(7.W), io.cin, 0.U(8.W))
        finalC := (carryBase & "hFEFEFEFF".U) | laneCin // 覆盖 bit8/16/24
    }.elsewhen(io.SIMDMode === "b10".U) {
        val laneCin = Cat(0.U(15.W), io.cin, 0.U(16.W))
        finalC := (carryBase & "hFFFEFFFF".U) | laneCin // 覆盖 bit16
    }.otherwise {
        finalC := carryBase
    }

    val res    = io.src1 ^ io.src2 ^ finalC
    val satRes = SatSIMD(io.src1, io.src2, res, io.SIMDMode)
    io.res := Mux(io.sat, satRes, res)

    // SIMD 下 cout 语义未完全定义，目前取最高位进位
    io.cout := c(0).asUInt(31)
}

object BLevelPAdder32_SIMD {
    def apply(src1: UInt, src2: UInt, cin: UInt, sat: Bool, SIMDMode: UInt): BLevelPAdder32_SIMD = {
        val adder = Module(new BLevelPAdder32_SIMD)
        adder.io.src1     := src1
        adder.io.src2     := src2
        adder.io.cin      := cin
        adder.io.sat      := sat
        adder.io.SIMDMode := SIMDMode
        adder
    }
}
