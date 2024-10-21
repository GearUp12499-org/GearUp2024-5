@file:SuppressLint("DefaultLocale")

package org.firstinspires.ftc.teamcode.mmooover.kinematics

import android.annotation.SuppressLint
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sqrt

private inline fun DoubleArray.mapInPlace(transform: (Double) -> Double) {
    for (i in this.indices) {
        this[i] = transform(this[i])
    }
}

data class CubicSpline(
    val a: Double,
    val b: Double,
    val c: Double,
    val d: Double,
    val lowX: Double,
    val highX: Double
) {
    companion object {
        fun fromArrayAndBounds(arr: DoubleArray, lowX: Double, highX: Double): CubicSpline {
            assert(arr.size == 4) { "Wrong size to import cubic spline from array : ${arr.size}" }
            return CubicSpline(
                arr[0], arr[1], arr[2], arr[3], lowX, highX
            )
        }
    }

    override fun toString() = "spline< ${toDesmos()} >"
    fun toDesmos(equation: Boolean = true, varSym: String = "x") = String.format(
        """%s%.6f$varSym^3 + %.6f$varSym^2 + %.6f$varSym + %.6f%s""",
        if (equation) {
            "y = "
        } else "", a, b, c, d,
        if (equation) {
            " \\left\\{%.6f \\leq $varSym \\leq %.6f\\right\\}".format(
                lowX, highX
            )
        } else ""
    )
}

data class CubicSplinePair(val x: CubicSpline, val y: CubicSpline) {
    fun toDesmos() = "(${x.toDesmos(false, "t")}, ${y.toDesmos(false, "t")})"
}

object CubicSplineSolver {
    fun solveMat(t1: Number, x1: Number, t2: Number, x2: Number, s1: Number, s2: Number) =
        solveMat(
            t1.toDouble(),
            x1.toDouble(),
            t2.toDouble(),
            x2.toDouble(),
            s1.toDouble(),
            s2.toDouble()
        )


    fun solveMat(
        t1: Double,
        x1: Double,
        t2: Double,
        x2: Double,
        s1: Double,
        s2: Double
    ): CubicSpline {
        var matrix: Array<DoubleArray> = arrayOf(
            doubleArrayOf(t1 * t1 * t1, t1 * t1, t1, 1.0, x1),
            doubleArrayOf(t2 * t2 * t2, t2 * t2, t2, 1.0, x2),
            doubleArrayOf(3 * t1 * t1, 2 * t1, 1.0, 0.0, s1),
            doubleArrayOf(3 * t2 * t2, 2 * t2, 1.0, 0.0, s2),
        )
        val nCols = matrix[0].size
        for ((rowN, row) in matrix.withIndex()) {
            val pivotCol = row.indexOfFirst { abs(it) > 0.001 }
            if (pivotCol == -1 || pivotCol == row.size - 1) throw IllegalStateException("No suitable pivot column found")
            val coeff = row[pivotCol]
            row.mapInPlace { it -> it / coeff }
            for ((rowN2, row2) in matrix.withIndex()) {
                if (rowN2 == rowN) continue
                val multiplier = row2[pivotCol]
                for (colIdx in 0 until nCols) {
                    row2[colIdx] -= multiplier * row[colIdx]
                }
            }
        }

        val factors = DoubleArray(4)
        for (index in 0..3) {
            factors[index] = matrix.first {
                abs(it[index] - 1.0) < 0.001
            }.last()
        }
        val result = CubicSpline.fromArrayAndBounds(factors, min(t1, t2), max(t1, t2))
        println(result.toString())
        return result
    }

    private fun getSlope(x1: Double, x2: Double, y1: Double, y2: Double) = (y2 - y1) / (x2 - x1)

    fun solveMultiSegment(xArr: DoubleArray, yArr: DoubleArray): List<CubicSpline> {
        assert(xArr.size == yArr.size) {
            "Mismatched coordinate sets: x has %d, y has %d".format(
                xArr.size,
                yArr.size
            )
        }
        assert(xArr.size >= 2) {
            "Not enough points (need at least 2)"
        }
        val m = DoubleArray(xArr.size)
        m[0] = getSlope(xArr[0], xArr[1], yArr[0], yArr[1])
        m[m.size - 1] = getSlope(
            xArr[xArr.size - 1],
            xArr[xArr.size - 2],
            yArr[yArr.size - 1],
            yArr[yArr.size - 2]
        )
        for (i in 1..m.size - 2) {
            m[i] = getSlope(
                xArr[i - 1],
                xArr[i + 1],
                yArr[i - 1],
                yArr[i + 1]
            )
        }
        val results: MutableList<CubicSpline> = mutableListOf()
        for (i in 0..<m.size - 1) results.add(
            solveMat(
                xArr[i], yArr[i], xArr[i + 1], yArr[i + 1],
                m[i], m[i + 1]
            )
        )
        return results
    }

    fun solve2DMultiSegment(xArr: DoubleArray, yArr: DoubleArray): List<CubicSplinePair> {
        assert(xArr.size == yArr.size) {
            "Mismatched coordinate sets: x has %d, y has %d".format(
                xArr.size,
                yArr.size
            )
        }
        val pointCount = xArr.size
        assert(pointCount >= 2) {
            "Not enough points (need at least 2)"
        }
        val timescale = DoubleArray(pointCount)
        var lastT = 0.0
        for (i in 1..<pointCount) {
            lastT += sqrt((xArr[i] - xArr[i - 1]).pow(2) + (yArr[i] - yArr[i - 1]).pow(2))
            timescale[i] = lastT
        }
        val xSplines = solveMultiSegment(timescale, xArr)
        val ySplines = solveMultiSegment(timescale, yArr)
        val result: MutableList<CubicSplinePair> = mutableListOf()
        for (i in 0..<xSplines.size) {
            result.add(
                CubicSplinePair(
                    xSplines[i],
                    ySplines[i]
                )
            )
        }
        return result
    }
}