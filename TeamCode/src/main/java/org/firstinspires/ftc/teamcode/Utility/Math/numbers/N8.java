package org.firstinspires.ftc.teamcode.Utility.Math.numbers;

import org.firstinspires.ftc.teamcode.Utility.Math.Nat;
import org.firstinspires.ftc.teamcode.Utility.Math.Num;

/**
 * A class representing the number {{ num }}.
 */
public final class N8 extends Num implements Nat<N8> {



    private N8() {
    }


    /**
     * The integer this class represents.
     *
     * @return The literal number {{ num }}.
     */
    @Override
    public int getNum() {
        return 8;
    }

    /**
     * The singleton instance of this class.
     */
    public static final N8 instance = new N8();
}
