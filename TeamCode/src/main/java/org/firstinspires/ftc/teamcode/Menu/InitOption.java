package org.firstinspires.ftc.teamcode.Menu;

import java.util.ArrayList;
import java.util.Collections;
import java.util.ListIterator;

class InitOption<T> {

    private T value;
    private ListIterator<T> current;
    private String name;
    private Mutable<T> var;

    InitOption(Mutable<T> variable, String name, T... args) {
        this.var = variable;
        this.name = name;

        ArrayList<T> values = new ArrayList<>();
        Collections.addAll(values, args);

        current = values.listIterator();
        while (current.hasNext()) {
            value = current.next();
            if (var.get() == value)
                break;
        }
    }

    public T selected() {
        return value;
    }

    public T next() {
        if (current.hasNext())
            value = current.next();
        return value;
    }

    public T prev() {
        if (current.hasPrevious())
            value = current.previous();
        return value;
    }

    public void apply() {
        var.set(value);
    }

    @Override
    public boolean equals(Object obj) {
        return var.equals(obj);
    }

    @Override
    public String toString() {
        return name;
    }
}
