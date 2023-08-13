package org.firstinspires.ftc.teamcode.Utility.Math;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.EigenOps_DDRM;
import org.ejml.simple.SimpleMatrix;

public final class DARE {
    private DARE() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Solves the discrete algebraic Riccati equation.
     *
     * @param A System matrix.
     * @param B Input matrix.
     * @param Q State cost matrix.
     * @param R Input cost matrix.
     * @return Solution of DARE.
     */
    public static SimpleMatrix dare(SimpleMatrix A, SimpleMatrix B, SimpleMatrix Q, SimpleMatrix R) {
        SimpleMatrix S = new SimpleMatrix(A.numRows(), A.numCols());
        WPIMathJNI.dare(
                A.getDDRM().getData(),
                B.getDDRM().getData(),
                Q.getDDRM().getData(),
                R.getDDRM().getData(),
                A.numCols(),
                B.numCols(),
                S.getDDRM().getData());
        return S;
    }

    /**
     * Solves the discrete algebraic Riccati equation.
     *
     * @param <States> Number of states.
     * @param <Inputs> Number of inputs.
     * @param A System matrix.
     * @param B Input matrix.
     * @param Q State cost matrix.
     * @param R Input cost matrix.
     * @return Solution of DARE.
     */
    public static <States extends Num, Inputs extends Num> Matrix<States, States> dare(
            Matrix<States, States> A,
            Matrix<States, Inputs> B,
            Matrix<States, States> Q,
            Matrix<Inputs, Inputs> R) {
        return new Matrix<>(dare(A.getStorage(), B.getStorage(), Q.getStorage(), R.getStorage()));
    }

    /**
     * Solves the discrete algebraic Riccati equation.
     *
     * @param A System matrix.
     * @param B Input matrix.
     * @param Q State cost matrix.
     * @param R Input cost matrix.
     * @param N State-input cross-term cost matrix.
     * @return Solution of DARE.
     */
    public static SimpleMatrix dare(
            SimpleMatrix A, SimpleMatrix B, SimpleMatrix Q, SimpleMatrix R, SimpleMatrix N) {
        // See
        // https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator#Infinite-horizon,_discrete-time_LQR
        // for the change of variables used here.
        SimpleMatrix scrA = A.minus(B.mult(R.solve(N.transpose())));
        SimpleMatrix scrQ = Q.minus(N.mult(R.solve(N.transpose())));

        SimpleMatrix S = new SimpleMatrix(A.numRows(), A.numCols());
        WPIMathJNI.dare(
                scrA.getDDRM().getData(),
                B.getDDRM().getData(),
                scrQ.getDDRM().getData(),
                R.getDDRM().getData(),
                A.numCols(),
                B.numCols(),
                S.getDDRM().getData());
        return S;
    }

    /**
     * Solves the discrete algebraic Riccati equation.
     *
     * @param <States> Number of states.
     * @param <Inputs> Number of inputs.
     * @param A System matrix.
     * @param B Input matrix.
     * @param Q State cost matrix.
     * @param R Input cost matrix.
     * @param N State-input cross-term cost matrix.
     * @return Solution of DARE.
     */
    public static <States extends Num, Inputs extends Num> Matrix<States, States> dare(
            Matrix<States, States> A,
            Matrix<States, Inputs> B,
            Matrix<States, States> Q,
            Matrix<Inputs, Inputs> R,
            Matrix<States, Inputs> N) {
        // This is a change of variables to make the DARE that includes Q, R, and N
        // cost matrices fit the form of the DARE that includes only Q and R cost
        // matrices.
        //
        // This is equivalent to solving the original DARE:
        //
        //   A₂ᵀXA₂ − X − A₂ᵀXB(BᵀXB + R)⁻¹BᵀXA₂ + Q₂ = 0
        //
        // where A₂ and Q₂ are a change of variables:
        //
        //   A₂ = A − BR⁻¹Nᵀ and Q₂ = Q − NR⁻¹Nᵀ
        return new Matrix<>(
                dare(
                        A.minus(B.times(R.solve(N.transpose()))).getStorage(),
                        B.getStorage(),
                        Q.minus(N.times(R.solve(N.transpose()))).getStorage(),
                        R.getStorage()));
    }
}
