package audio_receiver;

public interface audioFile extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "audio_receiver/audioFile";
  static final java.lang.String _DEFINITION = "string file_name\r\nint16[] file_data\r\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  java.lang.String getFileName();
  void setFileName(java.lang.String value);
  short[] getFileData();
  void setFileData(short[] value);
}
