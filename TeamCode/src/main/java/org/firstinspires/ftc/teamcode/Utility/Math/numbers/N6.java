package org.firstinspires.ftc.teamcode.Utility.Math.numbers;

import org.firstinspires.ftc.teamcode.Utility.Math.Nat;
import org.firstinspires.ftc.teamcode.Utility.Math.Num;

/**
 * A class representing the number {{ num }}.
 */
public final class N6 extends Num implements Nat<N6> {



    private N6() {
    }


    /**
     * The integer this class represents.
     *
     * @return The literal number {{ num }}.
     */
    @Override
    public int getNum() {
        return 6;
    }

    /**
     * The singleton instance of this class.
     */
    public static final N6 instance = new N6();
}
