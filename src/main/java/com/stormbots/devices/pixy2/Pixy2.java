package com.stormbots.devices.pixy2;

import java.nio.BufferUnderflowException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Pixy2 extends Subsystem{

  private short PIXY_SYNC_NOCHECKSUM = (short)0xc1ae;
  private short PIXY_SYNC_CHECKSUM = (short)0xc1af;

  private SPI spi;
  public Pixy2(SPI.Port port){
    spi = new SPI(port);
    spi.setClockRate(500_000); /// Pixy supports up to 2mbps
    spi.setMSBFirst();
    spi.setClockActiveLow(); 
    spi.setSampleDataOnTrailingEdge();
    spi.setChipSelectActiveLow();

    System.out.print("Setting up Pixy Cam ");
    System.out.println(getVersion());

  }

  public void initDefaultCommand(){
  }

  @Override
  public void periodic(){
  }

  public PixyPacket syncTransfer(ByteBuffer buffer){
    ByteBuffer input = ByteBuffer.allocate(40).order(ByteOrder.LITTLE_ENDIAN);
    //Flip the buffer if it wasn't done already
    if(buffer.limit()==buffer.position())buffer.flip(); 

    spi.write(buffer, buffer.limit());
    Timer.delay(0.0001); //wait for Pixy's guaranteed response time
    spi.read(true, input, input.remaining());
    return getNextPacket(input);
  }


  /**
   * Returns the next pixy response packet found in a buffer, if any
   * 
   * @param buffer that contains a pixy response packet
   * @return PixyPacket, with a correctly set `valid` property 
   */
  private PixyPacket getNextPacket(ByteBuffer buffer){
    PixyPacket packet = new PixyPacket();
    while(buffer.hasRemaining()){
      // Find and consume the start byte
      try{
        while(buffer.get()!=(byte)0xAF){/* Do Nothing */}
      }catch (BufferUnderflowException e){
        // System.err.println("Sync byte not found\n"+e);
        break;
      }

      //De-consume the last one, if the pixy stream stops emitting double 0xAF
      //// streambuffer.position(streambuffer.position()-1);

      // Verify Start Bytes
      if(buffer.getShort()!=(short)0xC1AF){continue;}

      //Get header info
      if(buffer.remaining()<4){continue;} //not enough bytes
      packet.type = buffer.get();
      packet.length = buffer.get();
      packet.checksum = buffer.getShort();

      //Get Data byte
      if(buffer.remaining()<packet.length){continue;}
      // packet.data.put(buffer.get(packet.length));
      for(int i=0; i<packet.length; i++){
        packet.data.put(buffer.get());
      }

      //prep data for use by other function
      packet.data.flip();
      packet.valid = true;
      packet.validateChecksum();
      return packet;
    }
    // Only get here when empty, so return a blank packet
    return packet;
  }

  private ByteBuffer getNewBuffer(int size){
    ByteBuffer buffer = ByteBuffer.allocate(size).order(ByteOrder.LITTLE_ENDIAN);
    return buffer;
  }

  public Version getVersion(){
    ByteBuffer output = getNewBuffer(4);
    output.putShort(PIXY_SYNC_NOCHECKSUM);
    output.put((byte)14);
    output.put((byte)0);
    
    PixyPacket response = syncTransfer(output);
    
    Version version = new Version();
    version.hardwareVersion = response.data.getShort();
    version.major = response.data.get();
    version.minor = response.data.get();
    version.firmware = response.data.getShort();
    version.firmwareType = response.data.getChar();

    return version;
  }

  public Block[] getBlocks(int signature, int numBlocksToReturn) {
    //look for a valid block in our buffer
    //if(not) return new Block[]{};
    
    ByteBuffer output = getNewBuffer(6);
    output.putShort(PIXY_SYNC_NOCHECKSUM);
    output.put((byte)32); //command
    output.put((byte)2);
    output.put((byte)signature);
    output.put((byte)numBlocksToReturn);

    PixyPacket response = syncTransfer(output);
    if(response.valid == false) return new Block[]{};

    Block block = new Block();
    block.xCenter = (int) response.data.getShort();
    block.yCenter = (int) response.data.getShort();
    block.width = (int) response.data.getShort();
    block.height = (int) response.data.getShort();
    block.colorAngle = (int) response.data.getShort();
    block.trackingIndex = response.data.get();
    block.age = response.data.get();

    Block[] blocks = new Block[]{block};
    return blocks;
  }

  public PixyPacket setLamp(boolean enableUpper, boolean enableLower) {
    ByteBuffer output = getNewBuffer(6);
    output.putShort(PIXY_SYNC_NOCHECKSUM);
    output.put((byte)22); //command
    output.put((byte)2); //num data bytes
    output.put((byte)(enableUpper?1:0));
    output.put((byte)(enableLower?1:0));

    return syncTransfer(output);
  }

  public Resolution getResolution() {
    ByteBuffer output = getNewBuffer(5);
    output.putShort(PIXY_SYNC_NOCHECKSUM);
    output.put((byte)12); //command
    output.put((byte)1); // num data bytes
    output.put((byte)0); // unused but required

    PixyPacket response = syncTransfer(output);
    
    Resolution resolution = new Resolution();
    resolution.width = (int) response.data.getShort();
    resolution.height = (int) response.data.getShort();

    return resolution;
  }

  public Line[] getMainFeatures(int featureType, int featureBitmap) {
    ByteBuffer output = getNewBuffer(6);
    output.putShort(PIXY_SYNC_NOCHECKSUM);
    output.put((byte)48); //command
    output.put((byte)2); // num data bytes
    output.put((byte)featureType); // 0 = main features 1 = all features
    output.put((byte)featureBitmap); // not sure what this means 


    PixyPacket response = syncTransfer(output);
    if(response.valid == false) return new Line[]{};

    Line line = new Line();
    response.data.get(); //feature type, discard
    line.featureLength = response.data.get();
    line.featureData = response.data.get();

    return new Line[]{line};
  }


  private void debugPrintBuffer(String label,ByteBuffer buffer){
    int cap = buffer.capacity();
    int limit = buffer.limit();
    int position = buffer.position();
    buffer.clear();

    System.out.printf("%s[%d..%d|%d]",label,position,limit,cap);

    for(int i = 0 ; i<cap ; i++){
      if(i==position)System.out.print('[');
      System.out.printf("%x.",buffer.get());
      if(i==limit-1)System.out.print(']');
    }
    buffer.position(position);
    buffer.limit(limit);
    System.out.println();
  }
}

