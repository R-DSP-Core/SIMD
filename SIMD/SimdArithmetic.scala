package zircon.simd

import chisel3._
import chisel3.util._

private class SimdAdderIO(width: Int) extends Bundle {
    val src1 = Input(UInt(width.W))
    val src2 = Input(UInt(width.W))
    val cin  = Input(UInt(1.W))
    val res  = Output(UInt(width.W))
    val cout = Output(UInt(1.W))
}

private object SimdCarry4 {
    def apply(p: UInt, g: UInt, cin: UInt): (Bool, Bool, UInt) = {
        require(p.getWidth == 4 && g.getWidth == 4)
        require(cin.getWidth == 1)

        val pn = p.andR
        val gn = g(3) | (p(3) & g(2)) | (p(3) & p(2) & g(1)) |
            (p(3) & p(2) & p(1) & g(0))
        val carry = Wire(Vec(4, Bool()))
        carry(0) := g(0) | (p(0) & cin)
        carry(1) := g(1) | (p(1) & g(0)) | (p(1) & p(0) & cin)
        carry(2) := g(2) | (p(2) & g(1)) | (p(2) & p(1) & g(0)) |
            (p(2) & p(1) & p(0) & cin)
        carry(3) := gn | (pn & cin)
        (pn, gn, carry.asUInt)
    }
}

/** 三十二位并行前缀加法器，按四位分组递推进位。 */
private class SimdPAdder32 extends Module {
    val io = IO(new SimdAdderIO(32))
    val bitPropagate = io.src1 | io.src2
    val bitGenerate  = io.src1 & io.src2

    val propagate = Wire(MixedVec(Vec(8, Bool()), Vec(2, Bool())))
    val generate  = Wire(MixedVec(Vec(8, Bool()), Vec(2, Bool())))
    val carry = Wire(MixedVec(
        Vec(8, UInt(4.W)),
        Vec(2, UInt(4.W)),
        Vec(1, UInt(4.W))))

    for (i <- 0 until 8) {
        val cin = if (i == 0) io.cin
            else if (i == 4) carry(2).asUInt(0)
            else carry(1).asUInt(i - 1)
        val (pn, gn, cn) = SimdCarry4(
            bitPropagate(i * 4 + 3, i * 4),
            bitGenerate(i * 4 + 3, i * 4),
            cin)
        propagate(0)(i) := pn
        generate(0)(i)  := gn
        carry(0)(i)     := cn
    }

    for (i <- 0 until 2) {
        val cin = if (i == 0) io.cin else carry(2).asUInt(i - 1)
        val (pn, gn, cn) = SimdCarry4(
            propagate(0).asUInt(i * 4 + 3, i * 4),
            generate(0).asUInt(i * 4 + 3, i * 4),
            cin)
        propagate(1)(i) := pn
        generate(1)(i)  := gn
        carry(1)(i)     := cn
    }

    val (_, _, topCarry) = SimdCarry4(
        0.U(2.W) ## propagate(1).asUInt,
        0.U(2.W) ## generate(1).asUInt,
        io.cin)
    carry(2)(0) := topCarry

    io.res  := io.src1 ^ io.src2 ^ (carry(0).asUInt(30, 0) ## io.cin)
    io.cout := carry(0).asUInt(31)
}

private class SimdPAdder64 extends Module {
    val io = IO(new SimdAdderIO(64))
    val bitPropagate = io.src1 | io.src2
    val bitGenerate  = io.src1 & io.src2

    val propagate = Wire(MixedVec(Vec(16, Bool()), Vec(4, Bool())))
    val generate  = Wire(MixedVec(Vec(16, Bool()), Vec(4, Bool())))
    val carry = Wire(MixedVec(
        Vec(16, UInt(4.W)),
        Vec(4, UInt(4.W)),
        Vec(1, UInt(4.W))))

    for (i <- 0 until 16) {
        val cin = if (i == 0) io.cin
            else if (i % 4 == 0) carry(2).asUInt(i / 4 - 1)
            else carry(1).asUInt(i - 1)
        val (pn, gn, cn) = SimdCarry4(
            bitPropagate(i * 4 + 3, i * 4),
            bitGenerate(i * 4 + 3, i * 4),
            cin)
        propagate(0)(i) := pn
        generate(0)(i)  := gn
        carry(0)(i)     := cn
    }

    for (i <- 0 until 4) {
        val cin = if (i == 0) io.cin else carry(2).asUInt(i - 1)
        val (pn, gn, cn) = SimdCarry4(
            propagate(0).asUInt(i * 4 + 3, i * 4),
            generate(0).asUInt(i * 4 + 3, i * 4),
            cin)
        propagate(1)(i) := pn
        generate(1)(i)  := gn
        carry(1)(i)     := cn
    }

    val (_, _, topCarry) = SimdCarry4(propagate(1).asUInt, generate(1).asUInt, io.cin)
    carry(2)(0) := topCarry

    io.res  := io.src1 ^ io.src2 ^ (carry(0).asUInt(62, 0) ## io.cin)
    io.cout := carry(0).asUInt(63)
}

private class SimdBooth2 extends Module {
    val io = IO(new Bundle {
        val src1 = Input(UInt(64.W))
        val src2 = Input(UInt(3.W))
        val res  = Output(UInt(64.W))
        val add1 = Output(Bool())
    })

    val code = WireDefault(0.U(64.W))
    switch(io.src2) {
        is(0.U) { code := 0.U }
        is(1.U) { code := io.src1 }
        is(2.U) { code := io.src1 }
        is(3.U) { code := io.src1(62, 0) ## 0.U(1.W) }
        is(4.U) { code := ~(io.src1(62, 0) ## 0.U(1.W)) }
        is(5.U) { code := ~io.src1 }
        is(6.U) { code := ~io.src1 }
        is(7.U) { code := 0.U }
    }
    io.res  := code
    io.add1 := io.src2(2) && !io.src2.andR
}

private object SimdCarrySaveAdder {
    def apply(src1: UInt, src2: UInt, src3: UInt): (UInt, UInt) = {
        val sum  = src1 ^ src2 ^ src3
        val cout = (src1 & src2) | (src1 & src3) | (src2 & src3)
        (sum, cout)
    }
}

private class SimdWallaceTree17Cin15 extends Module {
    val io = IO(new Bundle {
        val src  = Input(UInt(17.W))
        val cin  = Input(UInt(15.W))
        val sum  = Output(UInt(1.W))
        val cout = Output(UInt(16.W))
    })

    val sum = Wire(MixedVec(
        Vec(6, UInt(1.W)),
        Vec(4, UInt(1.W)),
        Vec(2, UInt(1.W)),
        Vec(2, UInt(1.W)),
        Vec(1, UInt(1.W)),
        Vec(1, UInt(1.W))))
    val cout = Wire(MixedVec(
        Vec(6, UInt(1.W)),
        Vec(4, UInt(1.W)),
        Vec(2, UInt(1.W)),
        Vec(2, UInt(1.W)),
        Vec(1, UInt(1.W)),
        Vec(1, UInt(1.W))))

    val src1 = MixedVecInit(
        VecInit(io.src(1), io.src(4), io.src(7), io.src(10), io.src(13), io.src(16)),
        VecInit(io.cin(0), io.cin(3), sum(0)(0), sum(0)(3)),
        VecInit(io.cin(6), sum(1)(1)),
        VecInit(io.cin(8), io.cin(11)),
        VecInit(io.cin(12)),
        VecInit(io.cin(13)))
    val src2 = MixedVecInit(
        VecInit(io.src(0), io.src(3), io.src(6), io.src(9), io.src(12), io.src(15)),
        VecInit(io.cin(1), io.cin(4), sum(0)(1), sum(0)(4)),
        VecInit(io.cin(7), sum(1)(2)),
        VecInit(io.cin(9), sum(2)(0)),
        VecInit(sum(3)(0)),
        VecInit(io.cin(14)))
    val src3 = MixedVecInit(
        VecInit(0.U, io.src(2), io.src(5), io.src(8), io.src(11), io.src(14)),
        VecInit(io.cin(2), io.cin(5), sum(0)(2), sum(0)(5)),
        VecInit(sum(1)(0), sum(1)(3)),
        VecInit(io.cin(10), sum(2)(1)),
        VecInit(sum(3)(1)),
        VecInit(sum(4)(0)))

    for (level <- 0 until 6; index <- 0 until sum(level).length) {
        val (s, c) = SimdCarrySaveAdder(src1(level)(index), src2(level)(index), src3(level)(index))
        sum(level)(index)  := s
        cout(level)(index) := c
    }

    io.sum  := sum(5).asUInt
    io.cout := cout.asUInt
}

private class SimdMul32IO extends Bundle {
    val src1 = Input(UInt(32.W))
    val src2 = Input(UInt(32.W))
    val res  = Output(UInt(32.W))
}

/** 有符号三十二乘三十二乘法器。运算分两级流水完成，对外只给出乘积的低三十二位。 */
private class SimdMul32 extends Module {
    val io = IO(new SimdMul32IO)

    def signExtend64(value: UInt): UInt = Fill(32, value(31)) ## value

    val src1 = signExtend64(io.src1)
    val src2 = signExtend64(io.src2)

    val add1Booth = Wire(Vec(17, Bool()))
    val partialProducts = Wire(Vec(17, UInt(64.W)))
    add1Booth(0) := src2(0)
    partialProducts(0) := Mux(src2(0), ~src1, 0.U)
    for (i <- 1 until 17) {
        val booth = Module(new SimdBooth2)
        booth.io.src1 := src1 << (2 * i - 1).U
        booth.io.src2 := src2(2 * i, 2 * i - 2)
        add1Booth(i) := booth.io.add1
        partialProducts(i) := booth.io.res
    }

    val add1Wallace = ShiftRegister(add1Booth, 1, VecInit.fill(17)(false.B), true.B)
    val productsWallace = ShiftRegister(
        partialProducts,
        1,
        VecInit.fill(17)(0.U(64.W)),
        true.B)
    val sumWallace  = Wire(Vec(64, UInt(1.W)))
    val coutWallace = Wire(Vec(64, UInt(16.W)))
    for (i <- 0 until 64) {
        val cin = if (i == 0) VecInit(add1Wallace.take(15)).asUInt else coutWallace(i - 1)(14, 0)
        val wallace = Module(new SimdWallaceTree17Cin15)
        wallace.io.src := VecInit(productsWallace.map(_(i))).asUInt
        wallace.io.cin := cin
        sumWallace(i)  := wallace.io.sum
        coutWallace(i) := wallace.io.cout
    }

    val finalSrc1 = ShiftRegister(sumWallace.asUInt, 1, 0.U, true.B)
    val finalSrc2 = ShiftRegister(
        VecInit(coutWallace.map(_(15))).asUInt(62, 0) ## add1Wallace(15),
        1,
        0.U,
        true.B)
    val finalCin = ShiftRegister(add1Wallace(16).asUInt, 1, 0.U, true.B)
    val finalAdder = Module(new SimdPAdder64)
    finalAdder.io.src1 := finalSrc1
    finalAdder.io.src2 := finalSrc2
    finalAdder.io.cin  := finalCin

    io.res := finalAdder.io.res(31, 0)
}
