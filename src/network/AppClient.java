import java.io.IOException;
import java.io.InputStreamReader;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.net.InetAddress;
import java.net.Socket;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.UUID;


public class AppClient {
    
     public static final String SERVER_ADDRESS = 
            //"192.168.0.13"; 
     "localhost";
     public static final int MAINPORTNUMBER = 1777;
     public static final int SERVER_UDP_PORT = 1776;
     public static final int APP_UDP_PORT = 1775;
     
     
     public static DatagramSocket datagramSocket;
     public static DatagramPacket dpClient;
     public static Socket socket;
     public static BufferedOutputStream streamToClient;
     public static BufferedReader fromClient;
     public static InetAddress serverIP;
     
     
     public static BlockingQueue<byte[]> udpOutQueue;
     public static BlockingQueue<byte[]>udpInQueue;
     
     public static BlockingQueue<String> tcpOutQueue;
     public static BlockingQueue<String> tcpInQueue;
     
     public static UDPReceiverThread udpReceiverThread;
     public static UDPSenderThread udpSenderThread;
     
     public static TCPReceiverThread tcpReceiverThread;
     public static TCPSenderThread tcpSenderThread;
     
     
     public static void main(String[] args){
         
         startConnection();
         
         UUID uuid = UUID.randomUUID();
         
         String id = uuid.toString();
         
         System.out.println("ID created: " + id);
         
         String response = "";
         
         while(!response.contains("connection_accepted")) {
        	 
        	 
        	 try {
        		 tcpOutQueue.add(("id:" + id));
        		 Thread.sleep(500);
        		 response = tcpInQueue.poll();
        		 if (response == null) response = "";
        		 else System.out.println("TCP response = "  + response);
        	 }catch(InterruptedException e) {
        		 System.out.println("interrupt in client:" + e.getMessage());
        	 }
        	 
         }
         
         System.out.println("Server has accepted connection");
         
         while(true){
             
             byte[] in = udpInQueue.poll();
             
             if (in != null){
                 String udp = new String(in);
                 
                 udp = udp.trim();
                 
                 System.out.println(udp);
                 
                 udpInQueue.clear();
                 
             }
             
             tcpOutQueue.add(("table_2|coke|"+id));
             
             try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                System.out.println("Error: app client interrupted");
                break;
            }
             
         }
         
     }
    
     public static void startConnection(){
         try {
             
             System.out.println("Creating TCP socket");
	     socket = new Socket(SERVER_ADDRESS, MAINPORTNUMBER);

             System.out.println("Creating UDP Socket");
             datagramSocket = new DatagramSocket(APP_UDP_PORT);
             
             System.out.println("Creating Streams");
             // Create input and output streams
             streamToClient = new BufferedOutputStream(socket.getOutputStream());
             fromClient = new BufferedReader(new InputStreamReader(socket.getInputStream()));
             
             serverIP = socket.getInetAddress();
             
             udpOutQueue = new LinkedBlockingQueue<>();
             udpInQueue = new LinkedBlockingQueue<>();
             
             tcpOutQueue = new LinkedBlockingQueue<>();
             tcpInQueue = new LinkedBlockingQueue<>();
             
	     System.out.println("Creating and starting UDP sender and receiver"); 
             udpReceiverThread = new UDPReceiverThread();
             udpSenderThread = new UDPSenderThread();
             udpReceiverThread.start();
             udpSenderThread.start();
             
             System.out.println("Creating and starting TCP sender and receiver");
             tcpReceiverThread = new TCPReceiverThread();
             tcpSenderThread = new TCPSenderThread();
             tcpReceiverThread.start();
             tcpSenderThread.start();
             
         } catch (IOException e) {
             e.printStackTrace();
             System.err.println("Error connecting to server");
         }
     }
     
     
     public static int udpAvailable(){
         return udpInQueue.size();
     }
     
     public static int tcpAvailable(){
         return tcpInQueue.size();
     }
     
     public static String getTCPMessage(){
         // will return null if the queue is empty
         return tcpInQueue.poll();
     }
     
     public static byte[] getUDPMessage(){
         // will return null if the queue is empty
         return udpInQueue.poll();
     }
     
     public static byte[][] getUDPMessages(){
         int size = udpInQueue.size();
         byte[][] allUpdates = new byte[size][];
         for(int i = 0; i < size; i++){
             allUpdates[i] = udpInQueue.poll();
         }
         return allUpdates;
     }
     
     public static String[] getTCPMessages(){
         int size = tcpInQueue.size();
         String[] allUpdates = new String[size];
         for(int i = 0; i < size; i++){
             allUpdates[i] = new String (tcpInQueue.poll());
         }
         return allUpdates;
     }
     
     public static boolean sendUDP(byte[] bytes){
         return udpOutQueue.offer(bytes);
     }
     
     public static boolean sendTCP(String message){
         return tcpOutQueue.offer(message);
     }
     
     private static class TCPReceiverThread extends Thread {

         public void run() {
        	 
        	 String stringFromUser;
        	 
        	 while(!Thread.interrupted()){
                 try {
                     stringFromUser = AppClient.fromClient.readLine();
                     tcpInQueue.offer(stringFromUser);
                 } catch (IOException e) {
                     System.out.println("Exception in Connected User: " + e.getMessage());
                     break;
                 }
             }
         }
     }
     
     private static class TCPSenderThread extends Thread{
         
         public void run(){
             String toSend;
             while(!Thread.interrupted()){
                 try {
                     toSend = AppClient.tcpOutQueue.take();
                     AppClient.streamToClient.write((toSend+"\n").getBytes());
                     AppClient.streamToClient.flush();
                     Thread.sleep(200);
                 } catch (InterruptedException e) {
                     e.printStackTrace();
                     System.out.println("Error: SenderThread: InterruptedException ");
                 } catch (IOException e) {
                     System.out.println("Error: SenderThread: IOException ");
                 }
                 
             }
         }
     }
     
     private static class UDPReceiverThread extends Thread{
         
         public void run(){
        	 
        	 byte[] data = new byte[64];
             
             DatagramPacket packet;
             System.out.println("listening for udps");
             while(!Thread.interrupted()){
                 data = new byte[128];
                 packet  = new DatagramPacket(data, data.length);
                 try {
                     AppClient.datagramSocket.receive(packet);
                     AppClient.udpInQueue.offer(packet.getData());
                 } catch (IOException e) {
                     System.out.println("Something went wrong when trying to recive packet in client.");
                 }
             }
         }
     }
     
     private static class UDPSenderThread extends Thread{
         
         public void run(){
             DatagramPacket packet;
             byte[] toSend;
             while(!Thread.interrupted()){
                 try {
                     toSend = AppClient.udpOutQueue.take();
                     //System.out.println("toSend:" + new String(toSend));
                     packet = new DatagramPacket(toSend, toSend.length,
                             AppClient.serverIP, AppClient.SERVER_UDP_PORT); 
                     AppClient.datagramSocket.send(packet);
                 } catch (InterruptedException e) {
                     e.printStackTrace();
                     System.out.println("Error: SenderThread: InterruptedException ");
                 } catch (IOException e) {
                     e.printStackTrace();
                     System.out.println("Error: SenderThread: IOException ");
                 }
                 
             }
         }
     }

  
 }
