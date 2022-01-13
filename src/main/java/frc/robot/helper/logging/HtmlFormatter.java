package frc.robot.helper.logging;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.Formatter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;


class HtmlFormatter extends Formatter {

    // this method is called for every log record
    public String format(LogRecord record) {

        StringBuilder buf = new StringBuilder();

        // Colorize the Rows based on Level of Log Record
        if (record.getLevel().intValue() >= Level.SEVERE.intValue()) {
            buf.append("<tr style=\"background-color:LightCoral\">\n");
            buf.append("\t<td style=\"color:Black\">");
            buf.append("<b>");
            buf.append(record.getLevel());
            buf.append("</b>");
        }
        else if (record.getLevel().intValue() >= Level.WARNING.intValue()) {
            buf.append("<tr style=\"background-color:LemonChiffon\">\n");
            buf.append("\t<td style=\"color:orange\">");
            buf.append("<b>");
            buf.append(record.getLevel());
            buf.append("</b>");
        }else if (record.getLevel().intValue() >= Level.INFO.intValue()){
            buf.append("\t<td style=\"color:darkslateblue\">");
            buf.append("<i>");
            buf.append(record.getLevel());
            buf.append("</i>");
        } else {
            buf.append("\t<td>");
            buf.append(record.getLevel());
        }

        // Date
        buf.append("</td>\n");
        buf.append("\t<td>");
        buf.append(calcDate(record.getMillis()));
        buf.append("</td>\n");

        // Class Path From Record
        buf.append("\t<td>");
        buf.append(record.getSourceClassName());
        buf.append("." );
        buf.append(record.getSourceMethodName());
        buf.append("</td>\n");

        // Log Message
        buf.append("\t<td>");
        buf.append(formatMessage(record));

        //Print Stack Trace If Available
        if (record.getThrown() != null) {
            buf.append("\n\t\t<pre>"); //<pre> Keeps Whitespace for Tabs and Newlines
            try {
                StringWriter sw = new StringWriter();
                PrintWriter pw = new PrintWriter(sw);
                record.getThrown().printStackTrace(pw);
                pw.close();

                buf.append(sw);
            } catch (Exception ex) {
                // ignore
            }
            buf.append("</pre>");
        }

        buf.append("\n</td>\n");
        buf.append("</tr>\n");

        return buf.toString();
    }

    private String calcDate(long millisecs) {
        SimpleDateFormat date_format = new SimpleDateFormat("MMM dd,yyyy HH:mm:ss");
        Date resultdate = new Date(millisecs);
        return date_format.format(resultdate);
    }

    // this method is called just after the handler using this
    // formatter is created
    public String getHead(Handler h) {
        return "<!DOCTYPE html>\n<head>\n<style>\n"
                + "table { width: 100; border-collapse: collapse }\n"
                + "th { font:bold 10pt Tahoma; }\n"
                + "td { font:normal 10pt Tahoma; }\n"
                + "h1 {font:bold 20pt Tahoma;}\n"
                + "tr:nth-child(even) {background-color: #f2f2f2;}\n"
                + "</style>\n"
                + "</head>\n"
                + "<body>\n"
                + "<h1> Robot Log</h1>\n"
                + "<p>" + (new Date()) + "</p>\n"
                + "<table table-layout=\"auto\" border=\"0\" cellpadding=\"5\" cellspacing=\"3\">\n"
                + "<tr align=\"left\">\n"
                + "\t<th style=\"width:10%\">Loglevel</th>\n"
                + "\t<th style=\"width:15%\">Time</th>\n"
                + "\t<th style=\"width:15%\">Source</th>\n"
                + "\t<th style=\"width:60%\">Log Message</th>\n"
                + "</tr>\n";
    }

    // this method is called just after the handler using this
    // formatter is closed
    public String getTail(Handler h) {
        return "</table>\n</body>\n</html>";
    }
}