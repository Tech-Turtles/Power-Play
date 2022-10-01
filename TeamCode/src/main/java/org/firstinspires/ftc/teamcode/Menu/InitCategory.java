package org.firstinspires.ftc.teamcode.Menu;

import java.util.ArrayList;
import java.util.ListIterator;

class InitCategory {

    private InitOption val;
    private String name;
    private ArrayList<InitOption> options;
    private ListIterator<InitOption> current;
    public int index = 0;

    public InitCategory(String name, ArrayList<InitOption> vars) {
        this.name = name;
        this.options = vars;
        val = options.get(0);
        current = options.listIterator();
    }

    public InitOption selected() {
        return val;
    }

    public InitOption next() {
        if(current.hasNext()) {
            val = current.next();
            index++;
        }
        return val;
    }

    public InitOption previous() {
        if(current.hasPrevious()) {
            val = current.previous();
            index--;
        }
        return val;
    }

    @Override
    public String toString() {
        return this.name;
    }

    public ArrayList<InitOption> getOptions() {
        return options;
    }
}