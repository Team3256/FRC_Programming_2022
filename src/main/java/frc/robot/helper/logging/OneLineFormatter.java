package frc.robot.helper.logging;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.Date;
import java.util.logging.Formatter;
import java.util.logging.Handler;
import java.util.logging.LogRecord;


public class OneLineFormatter extends Formatter {
    private static final String LINE_SEPARATOR = System.getProperty("line.separator");

    @Override
    public String format(LogRecord record) {
        StringBuilder buf = new StringBuilder();

        buf.append(new Date(record.getMillis()))
                .append(" ")
                .append(record.getLevel().getLocalizedName())
                .append(": ")
                .append(formatMessage(record))
                .append(LINE_SEPARATOR);

        //Print Stack Trace If Available
        if (record.getThrown() != null) {
            try {
                StringWriter sw = new StringWriter();
                PrintWriter pw = new PrintWriter(sw);
                record.getThrown().printStackTrace(pw);
                pw.close();
                buf.append(sw);
            } catch (Exception ex) {
                // ignore
            }
        }

        return buf.toString();
    }

    // this method is called just after the handler using this
    // formatter is created
    public String getHead(Handler h) {
        return "\n\n\n-------------------------\n" +
        "   Robot Code Restart   \n" +
        "-------------------------\n" +
        new Date() +
        "\n\n";
    }
}
