package org.firstinspires.ftc.teamcode.Utility;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryTools {

    public static String setFontColor(String color, String text) {
        return "<font color=\""+color+"\">"+text+"</font>";
    }

    public static String setHeader(int headerNumber, String text) {
        return "<h"+headerNumber+">"+text+"</h"+headerNumber+">";
    }

    /**
     * Dear god do I have too much time......
     *     Current working colors:
     *
     *     Red
     *     Yellow
     *     Fuchsia / Magenta
     *     Purple
     *     Lime
     *     Green
     *     Olive
     *     Teal
     *     Aqua / Cyan
     *     Blue
     *     Navy
     *     Maroon
     *     White
     *     LightGray / Silver
     *     DarkGray
     *     Gray
     */
    public void DisplayAllHTMLNameColors(Telemetry telemetry) {
        String builder = TelemetryTools.setHeader(1, "Colors") +
                TelemetryTools.setFontColor("INDIANRED", "INDIANRED") +
                "\n" +
                TelemetryTools.setFontColor("LIGHTCORAL", "LIGHTCORAL") +
                "\n" +
                TelemetryTools.setFontColor("SALMON", "SALMON") +
                "\n" +
                TelemetryTools.setFontColor("DARKSALMON", "DARKSALMON") +
                "\n" +
                TelemetryTools.setFontColor("LIGHTSALMON", "LIGHTSALMON") +
                "\n" +
                TelemetryTools.setFontColor("CRIMSON", "CRIMSON") +
                "\n" +
                TelemetryTools.setFontColor("RED", "RED") +
                "\n" +
                TelemetryTools.setFontColor("FIREBRICK", "FIREBRICK") +
                "\n" +
                TelemetryTools.setFontColor("DARKRED", "DARKRED") +
                "\n" +
                TelemetryTools.setFontColor("PINK", "PINK") +
                "\n" +
                TelemetryTools.setFontColor("LIGHTPINK", "LIGHTPINK") +
                "\n" +
                TelemetryTools.setFontColor("HOTPINK", "HOTPINK") +
                "\n" +
                TelemetryTools.setFontColor("DEEPPINK", "DEEPPINK") +
                "\n" +
                TelemetryTools.setFontColor("MEDIUMVIOLETRED", "MEDIUMVIOLETRED") +
                "\n" +
                TelemetryTools.setFontColor("PALEVIOLETRED", "PALEVIOLETRED") +
                "\n" +
                TelemetryTools.setFontColor("LIGHTSALMON", "LIGHTSALMON") +
                "\n" +
                TelemetryTools.setFontColor("CORAL", "CORAL") +
                "\n" +
                TelemetryTools.setFontColor("TOMATO", "TOMATO") +
                "\n" +
                TelemetryTools.setFontColor("ORANGERED", "ORANGERED") +
                "\n" +
                TelemetryTools.setFontColor("DARKORANGE", "DARKORANGE") +
                "\n" +
                TelemetryTools.setFontColor("ORANGE", "ORANGE") +
                "\n" +
                TelemetryTools.setFontColor("GOLD", "GOLD") +
                "\n" +
                TelemetryTools.setFontColor("YELLOW", "YELLOW") +
                "\n" +
                TelemetryTools.setFontColor("LIGHTYELLOW", "LIGHTYELLOW") +
                "\n" +
                TelemetryTools.setFontColor("LEMONCHIFFON", "LEMONCHIFFON") +
                "\n" +
                TelemetryTools.setFontColor("LIGHTGOLDENRODYELLOW", "LIGHTGOLDENRODYELLOW") +
                "\n" +
                "\n" +
                TelemetryTools.setFontColor("PAPAYAWHIP", "PAPAYAWHIP") +
                "\n" +
                "\n" +
                TelemetryTools.setFontColor("MOCCASIN", "MOCCASIN") +
                "\n" +
                TelemetryTools.setFontColor("PEACHPUFF", "PEACHPUFF") +
                "\n" +
                TelemetryTools.setFontColor("PALEGOLDENROD", "PALEGOLDENROD") +
                "\n" +
                TelemetryTools.setFontColor("KHAKI", "KHAKI") +
                "\n" +
                TelemetryTools.setFontColor("DARKKHAKI", "DARKKHAKI") +
                "\n" +
                TelemetryTools.setFontColor("LAVENDER", "LAVENDER") +
                "\n" +
                TelemetryTools.setFontColor("THISTLE", "THISTLE") +
                "\n" +
                TelemetryTools.setFontColor("PLUM", "PLUM") +
                "\n" +
                TelemetryTools.setFontColor("VIOLET", "VIOLET") +
                "\n" +
                TelemetryTools.setFontColor("ORCHID", "ORCHID") +
                "\n" +
                TelemetryTools.setFontColor("FUCHSIA", "FUCHSIA") +
                "\n" +
                TelemetryTools.setFontColor("MAGENTA", "MAGENTA") +
                "\n" +
                TelemetryTools.setFontColor("MEDIUMORCHID", "MEDIUMORCHID") +
                "\n" +
                TelemetryTools.setFontColor("MEDIUMPURPLE", "MEDIUMPURPLE") +
                "\n" +
                TelemetryTools.setFontColor("REBECCAPURPLE", "REBECCAPURPLE") +
                "\n" +
                TelemetryTools.setFontColor("BLUEVIOLET", "BLUEVIOLET") +
                "\n" +
                TelemetryTools.setFontColor("DARKVIOLET", "DARKVIOLET") +
                "\n" +
                TelemetryTools.setFontColor("DARKORCHID", "DARKORCHID") +
                "\n" +
                TelemetryTools.setFontColor("DARKMAGENTA", "DARKMAGENTA") +
                "\n" +
                TelemetryTools.setFontColor("PURPLE", "PURPLE") +
                "\n" +
                TelemetryTools.setFontColor("INDIGO", "INDIGO") +
                "\n" +
                TelemetryTools.setFontColor("SLATEBLUE", "SLATEBLUE") +
                "\n" +
                TelemetryTools.setFontColor("DARKSLATEBLUE", "DARKSLATEBLUE") +
                "\n" +
                TelemetryTools.setFontColor("MEDIUMSLATEBLUE", "MEDIUMSLATEBLUE") +
                "\n" +
                TelemetryTools.setFontColor("GREENYELLOW", "GREENYELLOW") +
                "\n" +
                TelemetryTools.setFontColor("CHARTREUSE", "CHARTREUSE") +
                "\n" +
                TelemetryTools.setFontColor("LAWNGREEN", "LAWNGREEN") +
                "\n" +
                TelemetryTools.setFontColor("LIME", "LIME") +
                "\n" +
                TelemetryTools.setFontColor("LIMEGREEN", "LIMEGREEN") +
                "\n" +
                TelemetryTools.setFontColor("PALEGREEN", "PALEGREEN") +
                "\n" +
                TelemetryTools.setFontColor("LIGHTGREEN", "LIGHTGREEN") +
                "\n" +
                TelemetryTools.setFontColor("MEDIUMSPRINGGREEN", "MEDIUMSPRINGGREEN") +
                "\n" +
                TelemetryTools.setFontColor("SPRINGGREEN", "SPRINGGREEN") +
                "\n" +
                TelemetryTools.setFontColor("MEDIUMSEAGREEN", "MEDIUMSEAGREEN") +
                "\n" +
                TelemetryTools.setFontColor("SEAGREEN", "SEAGREEN") +
                "\n" +
                TelemetryTools.setFontColor("FORESTGREEN", "FORESTGREEN") +
                "\n" +
                TelemetryTools.setFontColor("GREEN", "GREEN") +
                "\n" +
                TelemetryTools.setFontColor("DARKGREEN", "DARKGREEN") +
                "\n" +
                TelemetryTools.setFontColor("YELLOWGREEN", "YELLOWGREEN") +
                "\n" +
                TelemetryTools.setFontColor("OLIVEDRAB", "OLIVEDRAB") +
                "\n" +
                TelemetryTools.setFontColor("OLIVE", "OLIVE") +
                "\n" +
                TelemetryTools.setFontColor("DARKOLIVEGREEN", "DARKOLIVEGREEN") +
                "\n" +
                TelemetryTools.setFontColor("MEDIUMAQUAMARINE", "MEDIUMAQUAMARINE") +
                "\n" +
                TelemetryTools.setFontColor("DARKSEAGREEN", "DARKSEAGREEN") +
                "\n" +
                TelemetryTools.setFontColor("LIGHTSEAGREEN", "LIGHTSEAGREEN") +
                "\n" +
                TelemetryTools.setFontColor("DARKCYAN", "DARKCYAN") +
                "\n" +
                TelemetryTools.setFontColor("TEAL", "TEAL") +
                "\n" +
                TelemetryTools.setFontColor("AQUA", "AQUA") +
                "\n" +
                TelemetryTools.setFontColor("CYAN", "CYAN") +
                "\n" +
                TelemetryTools.setFontColor("LIGHTCYAN", "LIGHTCYAN") +
                "\n" +
                TelemetryTools.setFontColor("PALETURQUOISE", "PALETURQUOISE") +
                "\n" +
                TelemetryTools.setFontColor("AQUAMARINE", "AQUAMARINE") +
                "\n" +
                TelemetryTools.setFontColor("TURQUOISE", "TURQUOISE") +
                "\n" +
                TelemetryTools.setFontColor("MEDIUMTURQUOISE", "MEDIUMTURQUOISE") +
                "\n" +
                TelemetryTools.setFontColor("DARKTURQUOISE", "DARKTURQUOISE") +
                "\n" +
                TelemetryTools.setFontColor("CADETBLUE", "CADETBLUE") +
                "\n" +
                TelemetryTools.setFontColor("STEELBLUE", "STEELBLUE") +
                "\n" +
                TelemetryTools.setFontColor("LIGHTSTEELBLUE", "LIGHTSTEELBLUE") +
                "\n" +
                TelemetryTools.setFontColor("POWDERBLUE", "POWDERBLUE") +
                "\n" +
                TelemetryTools.setFontColor("LIGHTBLUE", "LIGHTBLUE") +
                "\n" +
                TelemetryTools.setFontColor("SKYBLUE", "SKYBLUE") +
                "\n" +
                TelemetryTools.setFontColor("LIGHTSKYBLUE", "LIGHTSKYBLUE") +
                "\n" +
                TelemetryTools.setFontColor("DEEPSKYBLUE", "DEEPSKYBLUE") +
                "\n" +
                TelemetryTools.setFontColor("DODGERBLUE", "DODGERBLUE") +
                "\n" +
                TelemetryTools.setFontColor("CORNFLOWERBLUE", "CORNFLOWERBLUE") +
                "\n" +
                TelemetryTools.setFontColor("MEDIUMSLATEBLUE", "MEDIUMSLATEBLUE") +
                "\n" +
                TelemetryTools.setFontColor("ROYALBLUE", "ROYALBLUE") +
                "\n" +
                TelemetryTools.setFontColor("BLUE", "BLUE") +
                "\n" +
                TelemetryTools.setFontColor("MEDIUMBLUE", "MEDIUMBLUE") +
                "\n" +
                TelemetryTools.setFontColor("DARKBLUE", "DARKBLUE") +
                "\n" +
                TelemetryTools.setFontColor("NAVY", "NAVY") +
                "\n" +
                TelemetryTools.setFontColor("MIDNIGHTBLUE", "MIDNIGHTBLUE") +
                "\n" +
                TelemetryTools.setFontColor("CORNSILK", "CORNSILK") +
                "\n" +
                TelemetryTools.setFontColor("BLANCHEDALMOND", "BLANCHEDALMOND") +
                "\n" +
                TelemetryTools.setFontColor("BISQUE", "BISQUE") +
                "\n" +
                TelemetryTools.setFontColor("NAVAJOWHITE", "NAVAJOWHITE") +
                "\n" +
                TelemetryTools.setFontColor("WHEAT", "WHEAT") +
                "\n" +
                TelemetryTools.setFontColor("BURLYWOOD", "BURLYWOOD") +
                "\n" +
                TelemetryTools.setFontColor("TAN", "TAN") +
                "\n" +
                TelemetryTools.setFontColor("ROSYBROWN", "ROSYBROWN") +
                "\n" +
                TelemetryTools.setFontColor("SANDYBROWN", "SANDYBROWN") +
                "\n" +
                TelemetryTools.setFontColor("GOLDENROD", "GOLDENROD") +
                "\n" +
                TelemetryTools.setFontColor("DARKGOLDENROD", "DARKGOLDENROD") +
                "\n" +
                TelemetryTools.setFontColor("PERU", "PERU") +
                "\n" +
                TelemetryTools.setFontColor("CHOCOLATE", "CHOCOLATE") +
                "\n" +
                TelemetryTools.setFontColor("SADDLEBROWN", "SADDLEBROWN") +
                "\n" +
                TelemetryTools.setFontColor("SIENNA", "SIENNA") +
                "\n" +
                TelemetryTools.setFontColor("BROWN", "BROWN") +
                "\n" +
                TelemetryTools.setFontColor("MAROON", "MAROON") +
                "\n" +
                TelemetryTools.setFontColor("WHITE", "WHITE") +
                "\n" +
                TelemetryTools.setFontColor("SNOW", "SNOW") +
                "\n" +
                TelemetryTools.setFontColor("HONEYDEW", "HONEYDEW") +
                "\n" +
                TelemetryTools.setFontColor("MINTCREAM", "MINTCREAM") +
                "\n" +
                TelemetryTools.setFontColor("AZURE", "AZURE") +
                "\n" +
                TelemetryTools.setFontColor("ALICEBLUE", "ALICEBLUE") +
                "\n" +
                TelemetryTools.setFontColor("GHOSTWHITE", "GHOSTWHITE") +
                "\n" +
                TelemetryTools.setFontColor("WHITESMOKE", "WHITESMOKE") +
                "\n" +
                TelemetryTools.setFontColor("SEASHELL", "SEASHELL") +
                "\n" +
                TelemetryTools.setFontColor("BEIGE", "BEIGE") +
                "\n" +
                TelemetryTools.setFontColor("OLDLACE", "OLDLACE") +
                "\n" +
                TelemetryTools.setFontColor("FLORALWHITE", "FLORALWHITE") +
                "\n" +
                TelemetryTools.setFontColor("IVORY", "IVORY") +
                "\n" +
                TelemetryTools.setFontColor("ANTIQUEWHITE", "ANTIQUEWHITE") +
                "\n" +
                TelemetryTools.setFontColor("LINEN", "LINEN") +
                "\n" +
                TelemetryTools.setFontColor("LAVENDERBLUSH", "LAVENDERBLUSH") +
                "\n" +
                TelemetryTools.setFontColor("MISTYROSE", "MISTYROSE") +
                "\n" +
                TelemetryTools.setFontColor("GAINSBORO", "GAINSBORO") +
                "\n" +
                TelemetryTools.setFontColor("LIGHTGRAY", "LIGHTGRAY") +
                "\n" +
                TelemetryTools.setFontColor("SILVER", "SILVER") +
                "\n" +
                TelemetryTools.setFontColor("DARKGRAY", "DARKGRAY") +
                "\n" +
                TelemetryTools.setFontColor("GRAY", "GRAY") +
                "\n" +
                TelemetryTools.setFontColor("DIMGRAY", "DIMGRAY") +
                "\n" +
                TelemetryTools.setFontColor("LIGHTSLATEGRAY", "LIGHTSLATEGRAY") +
                "\n" +
                TelemetryTools.setFontColor("SLATEGRAY", "SLATEGRAY") +
                "\n" +
                TelemetryTools.setFontColor("DARKSLATEGRAY", "DARKSLATEGRAY") +
                "\n" +
                TelemetryTools.setFontColor("BLACK", "BLACK");
        telemetry.addLine(builder);
    }
}
