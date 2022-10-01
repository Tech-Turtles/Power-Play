package org.firstinspires.ftc.teamcode.HardwareTypes;

public enum ExpansionHubs {

    HUB1("Expansion Hub 173"),
    HUB2("Expansion Hub 2");

    String name;

    ExpansionHubs(String name) {
        this.name = name;
    }

    public String getHub() {
        return name;
    }
}