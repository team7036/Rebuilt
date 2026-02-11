package frc.robot.hardware;

public interface AbsoluteEncoder extends Encoder {
    /**
     * Applies the offset for the given absolute encoder
     * @param offset offset in radians
     */
    void setOffset(double offset);
}
