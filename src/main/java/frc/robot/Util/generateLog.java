package frc.robot.Util;
import java.io.File;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.sql.Timestamp;
import java.io.FileWriter;

public class generateLog {
    Charset utf8 = StandardCharsets.UTF_8;
    Timestamp timestamp = new Timestamp(System.currentTimeMillis());
    String DirectoryPath = "C:/Users/FRC3467/Desktop/Logs/";
    File m_file = new File(DirectoryPath + timestamp + ".txt"); // Create a new file that is named after a timestamp
    Path m_path = Path.of(DirectoryPath + timestamp + ".txt");

    public void inputData(String subsystem, String category, String data) {
        /////////////////////////////////////////////////
        //
        //
        //
        //
        /////////////////////////////////////////////////
        Timestamp curTimeStamp = new Timestamp(System.currentTimeMillis());
        //FileWriter writeTo = new FileWriter("Test");
        //Files.writeString(Paths.get(""), curTimeStamp + "    " + subsystem + "    " + category + "    " + data + "\n", utf8);
        //Files.writeString(m_path, test);
    }
}
