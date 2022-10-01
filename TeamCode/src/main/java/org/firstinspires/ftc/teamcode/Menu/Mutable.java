package org.firstinspires.ftc.teamcode.Menu;

public class Mutable<T> {
    private T var;

    public Mutable(T initial)
    {
        this.var = initial;
    }

    public T get()
    {
        return var;
    }

    public void set(T var)
    {
        this.var = var;
    }

    public String toString()
    {
        return var.toString();
    }
}
