package zircon.simd.packsimd

import chisel3._
import chisel3.util._

/** SIMDMode: 00/11=1x32, 01=4x8, 10=2x16。 */
class MulIO(n: Int) extends Bundle {
    val src1     = Input(UInt(n.W))
    val src2     = Input(UInt(n.W))
    val SIMDMode = Input(UInt(2.W))
    val res      = Output(UInt(n.W))
}

/** 3:2 CSA，carry 左移 1 位。 */
object PackCsa3 {
    def apply(a: UInt, b: UInt, c: UInt): (UInt, UInt) = {
        val w     = a.getWidth
        val sum   = a ^ b ^ c
        val carry = (((a & b) | (a & c) | (b & c)) << 1)(w - 1, 0)
        (sum, carry)
    }
}

object PackCsaReduce {
    def apply(terms: Seq[UInt], width: Int): (UInt, UInt) = {
        require(terms.nonEmpty)
        var layer: Seq[UInt] = terms.map(t => t.pad(width))
        while (layer.length > 2) {
            val next = scala.collection.mutable.ArrayBuffer[UInt]()
            var i = 0
            while (i + 2 < layer.length) {
                val (s, c) = PackCsa3(layer(i), layer(i + 1), layer(i + 2))
                next += s
                next += c
                i += 3
            }
            while (i < layer.length) {
                next += layer(i)
                i += 1
            }
            layer = next.toSeq
        }
        if (layer.length == 1) (layer.head, 0.U(width.W))
        else (layer(0), layer(1))
    }
}

class PackPrefixAdder16 extends Module {
    val io = IO(new Bundle {
        val src1 = Input(UInt(16.W))
        val src2 = Input(UInt(16.W))
        val cin  = Input(UInt(1.W))
        val res  = Output(UInt(16.W))
    })
    val pi = io.src1 | io.src2
    val gi = io.src1 & io.src2

    val p0 = Wire(Vec(4, UInt(1.W)))
    val g0 = Wire(Vec(4, UInt(1.W)))
    val c0 = Wire(Vec(4, UInt(4.W)))
    val c1 = Wire(Vec(1, UInt(4.W)))

    for (i <- 0 until 4) {
        val cin = if (i == 0) io.cin else c1.asUInt(i - 1)
        val (pn, gn, cn) = BLevelCarry4(pi(i * 4 + 3, i * 4), gi(i * 4 + 3, i * 4), cin)
        p0(i) := pn
        g0(i) := gn
        c0(i) := cn
    }
    val (_, _, c1n) = BLevelCarry4(p0.asUInt, g0.asUInt, io.cin)
    c1(0) := c1n

    io.res := io.src1 ^ io.src2 ^ (c0.asUInt(14, 0) ## io.cin)
}

class PackPrefixAdder64 extends Module {
    val io = IO(new Bundle {
        val src1 = Input(UInt(64.W))
        val src2 = Input(UInt(64.W))
        val cin  = Input(UInt(1.W))
        val res  = Output(UInt(64.W))
    })
    val pi = io.src1 | io.src2
    val gi = io.src1 & io.src2

    val p0 = Wire(Vec(16, UInt(1.W)))
    val g0 = Wire(Vec(16, UInt(1.W)))
    val c0 = Wire(Vec(16, UInt(4.W)))
    val p1 = Wire(Vec(4, UInt(1.W)))
    val g1 = Wire(Vec(4, UInt(1.W)))
    val c1 = Wire(Vec(4, UInt(4.W)))
    val c2 = Wire(Vec(1, UInt(4.W)))

    for (i <- 0 until 16) {
        val cin =
            if (i == 0) io.cin
            else if (i % 4 == 0) c2.asUInt(i / 4 - 1)
            else c1.asUInt(i - 1)
        val (pn, gn, cn) = BLevelCarry4(pi(i * 4 + 3, i * 4), gi(i * 4 + 3, i * 4), cin)
        p0(i) := pn
        g0(i) := gn
        c0(i) := cn
    }
    for (i <- 0 until 4) {
        val cin = if (i == 0) io.cin else c2.asUInt(i - 1)
        val (pn, gn, cn) = BLevelCarry4(p0.asUInt(i * 4 + 3, i * 4), g0.asUInt(i * 4 + 3, i * 4), cin)
        p1(i) := pn
        g1(i) := gn
        c1(i) := cn
    }
    val (_, _, c2n) = BLevelCarry4(p1.asUInt, g1.asUInt, io.cin)
    c2(0) := c2n

    io.res := io.src1 ^ io.src2 ^ (c0.asUInt(62, 0) ## io.cin)
}

object PackPrefixAdder16 {
    def apply(src1: UInt, src2: UInt, cin: UInt): UInt = {
        val adder = Module(new PackPrefixAdder16)
        adder.io.src1 := src1
        adder.io.src2 := src2
        adder.io.cin  := cin
        adder.io.res
    }
}

object PackPrefixAdder64 {
    def apply(src1: UInt, src2: UInt, cin: UInt): UInt = {
        val adder = Module(new PackPrefixAdder64)
        adder.io.src1 := src1
        adder.io.src2 := src2
        adder.io.cin  := cin
        adder.io.res
    }
}

/** 无符号 8x8 radix-4 Booth；负编码对部分积按位取反，并在 bit0 加 1。 */
class Mul8 extends Module {
    val io = IO(new Bundle {
        val a   = Input(UInt(8.W))
        val b   = Input(UInt(8.W))
        val res = Output(UInt(16.W))
    })

    val aExt = 0.U(8.W) ## io.a
    val y    = Cat(0.U(2.W), io.b, 0.U(1.W)) // y[-1]=0, y[9:8]=0

    val pp   = Wire(Vec(5, UInt(16.W)))
    val add1 = Wire(Vec(5, Bool()))

    for (k <- 0 until 5) {
        val rec = y(2 * k + 2, 2 * k)
        val mag = WireDefault(0.U(16.W))
        val neg = WireDefault(false.B)
        switch(rec) {
            is("b000".U, "b111".U) { mag := 0.U; neg := false.B }
            is("b001".U, "b010".U) { mag := (aExt << (2 * k).U)(15, 0); neg := false.B }
            is("b011".U)           { mag := (aExt << (2 * k + 1).U)(15, 0); neg := false.B }
            is("b100".U)           { mag := (aExt << (2 * k + 1).U)(15, 0); neg := true.B }
            is("b101".U, "b110".U) { mag := (aExt << (2 * k).U)(15, 0); neg := true.B }
        }
        pp(k)   := Mux(neg, ~mag, mag)
        add1(k) := neg
    }

    val terms: Seq[UInt] =
        pp.toSeq ++ add1.map(a1 => Mux(a1, 1.U(16.W), 0.U(16.W))).toSeq
    val (s, c) = PackCsaReduce(terms, 16)
    io.res := PackPrefixAdder16(s, c, 0.U)
}

object Mul8 {
    def apply(a: UInt, b: UInt): UInt = {
        val m = Module(new Mul8)
        m.io.a := a
        m.io.b := b
        m.io.res
    }
}

/** 16×Mul8 嵌套阵列；按 SIMDMode 屏蔽交叉项，各 lane 输出乘积低半。 */
class BLevelPMul32_SIMD extends Module {
    val io = IO(new MulIO(32))

    val aLimb = Wire(Vec(4, UInt(8.W)))
    val bLimb = Wire(Vec(4, UInt(8.W)))
    for (i <- 0 until 4) {
        aLimb(i) := io.src1(i * 8 + 7, i * 8)
        bLimb(i) := io.src2(i * 8 + 7, i * 8)
    }

    val p = Wire(Vec(4, Vec(4, UInt(16.W))))
    for (i <- 0 until 4; j <- 0 until 4) {
        p(i)(j) := Mul8(aLimb(i), bLimb(j))
    }

    val mode8  = io.SIMDMode === "b01".U
    val mode16 = io.SIMDMode === "b10".U

    val aligned = Wire(Vec(16, UInt(64.W)))
    for (i <- 0 until 4; j <- 0 until 4) {
        val enableDiag = (i == j).B
        val enable16   = ((i <= 1) && (j <= 1)).B || ((i >= 2) && (j >= 2)).B
        val enable     = Mux(mode8, enableDiag, Mux(mode16, enable16, true.B))
        val raw        = Mux(enable, p(i)(j), 0.U(16.W))
        aligned(i * 4 + j) := (0.U(48.W) ## raw) << (8 * (i + j)).U
    }
    val res8Comb = Cat(p(3)(3)(7, 0), p(2)(2)(7, 0), p(1)(1)(7, 0), p(0)(0)(7, 0))

    // S1：对齐部分积 / 4x8 对角结果 / 模式（对应主树 Booth → Wallace 切级）
    val alignedS1 = ShiftRegister(aligned, 1, VecInit.fill(16)(0.U(64.W)), true.B)
    val res8S1    = ShiftRegister(res8Comb, 1, 0.U(32.W), true.B)
    val modeS1    = ShiftRegister(io.SIMDMode, 1, 0.U(2.W), true.B)

    val (sumS, sumC) = PackCsaReduce(alignedS1.toSeq, 64)

    // S2：CSA 压缩结果（对应主树 Wallace → CPA 切级）
    val sumSS2 = ShiftRegister(sumS, 1, 0.U(64.W), true.B)
    val sumCS2 = ShiftRegister(sumC, 1, 0.U(64.W), true.B)
    val res8S2 = ShiftRegister(res8S1, 1, 0.U(32.W), true.B)
    val modeS2 = ShiftRegister(modeS1, 1, 0.U(2.W), true.B)

    val sum64 = PackPrefixAdder64(sumSS2, sumCS2, 0.U)
    val res16 = Cat(sum64(47, 32), sum64(15, 0))
    val res32 = sum64(31, 0)

    io.res := Mux(
        modeS2 === "b01".U,
        res8S2,
        Mux(modeS2 === "b10".U, res16, res32)
    )
}

object BLevelPMul32_SIMD {
    def apply(src1: UInt, src2: UInt, SIMDMode: UInt): BLevelPMul32_SIMD = {
        val m = Module(new BLevelPMul32_SIMD)
        m.io.src1     := src1
        m.io.src2     := src2
        m.io.SIMDMode := SIMDMode
        m
    }
}
