

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.util.Iterator;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ConcurrentLinkedQueue;


public class AWSRobotServer{
    
    public static int TCPport = 1777;
    public static int UDPport = 1776;
    public static int APP_UDP_PORT = 1775;
    
    private DatagramSocket datagramSocket;
    private BlockingQueue<DatagramPacket> packetQueue;
    
    private ConnectedClient robot;
    private ConcurrentLinkedQueue<ConnectedClient> customers;

    public AWSRobotServer(int port) throws SocketException {
        datagramSocket = new DatagramSocket(port);
        packetQueue = new LinkedBlockingQueue<>();
        customers = new ConcurrentLinkedQueue<>();
    }
    
    public static void main(String[] args) {

        System.out.println("Starting server");
        
        AWSRobotServer server = null;
        ServerSocket serverMainSocket = null;
        ServerDatagramListener datagramListener = null;
        ServerDatagramHandler datagramHandler = null;
        
        ServerWorker worker = null;
        
        try {
            serverMainSocket = new ServerSocket(TCPport);
            server = new AWSRobotServer(UDPport);
            
            datagramListener = new ServerDatagramListener(server);
            datagramHandler = new ServerDatagramHandler(server);
            worker =  new ServerWorker(server);
            
            
            datagramListener.start();
            datagramHandler.start();
            worker.start();
            
        }catch (IOException e) {
            e.printStackTrace();
            System.err.println("Couldn't listen on port " + TCPport);
            System.exit(1);
        }
       
        
        System.out.println("Starting main loop");
        try {
            // Loop forever listening for new clients
            
            while (true) {

                // wait for next player to connect
                Socket socket = serverMainSocket.accept();

                System.out.println("Someone has connected, creating streams");
                
                // Create input and output streams
                BufferedOutputStream streamToClient = new BufferedOutputStream(socket.getOutputStream());
                BufferedReader fromClient = new BufferedReader(new InputStreamReader(socket.getInputStream()));

                
                System.out.println("streams created");

                (new ConnectedClient(socket, server, streamToClient, fromClient)).start();

            }
        } catch (IOException e) {
            System.err.println("IO error " + e.getMessage());

        } finally {
            try {
                serverMainSocket.close();
                datagramListener.interrupt();
                datagramHandler.interrupt();;
            } catch (Exception e) {
                // Nothing to do
            }
        }

    }
    
    
    public boolean offerPacket(DatagramPacket p){
        return packetQueue.offer(p);
    }
    
    public DatagramPacket getPacket() throws InterruptedException{
        return packetQueue.take();
    }

    public DatagramSocket getDatagramSocket() {
        return datagramSocket;
    }
    
    public ConnectedClient getRobot(){
        return robot;
    }
    
    public ConcurrentLinkedQueue<ConnectedClient> getCustomers(){
        return customers;
    }
    
    public void addCustomer(ConnectedClient c){
        customers.add(c);
        System.out.println("cust added: new size=" + customers.size());
    }
    
    public void removeCustomer(ConnectedClient c){
        customers.remove(c);
    }
    
    public void setRobot(ConnectedClient r){
        robot = r;
    }
    
    public void removeRobot(){
        robot = null;
    }
    
    public void sendUDPMessageToCustomers(String m){
        
    	//System.out.println("sending UDP to all customers| customerSize:" + customers.size() );
    	
        Iterator<ConnectedClient> i = customers.iterator();
        
        //System.out.println("new size: " + customers.size() );
        
        while(i.hasNext()){
            i.next().sendUDP(m);
        }
        
    }
    
    public void sendTCPMessageToCustomers(String m){
    	
        Iterator<ConnectedClient> i = customers.iterator();
        
        while(i.hasNext()){
            i.next().sendTCP(m);
        }
        
    }
    
    public void handleCustomerMessage(String sender, String m){
        System.out.println("Error: received UDP from customer: " + sender + " - " + m);
    }
    
    private static class ServerDatagramListener extends Thread{
        
        private AWSRobotServer server;

        public ServerDatagramListener(AWSRobotServer server){
            this.server = server;
        }
        
        public void run(){
        
            byte[] data;
            
            DatagramPacket packet;
            
            while(!Thread.interrupted()){
                data = new byte[64];
                packet  = new DatagramPacket(data, data.length);
                try {
                    server.getDatagramSocket().receive(packet);
                } catch (IOException e) {
                    System.out.println("Something went wrong when trying to recive packet in server.");
                }
                if (packet.getData()[0] == 126) {
                    // 126 = "~" the special character used to
                    // identify a message from client to server
                    // (without this the server will try to receive all
                    // udp messages it sends when testing on localhost)
                    server.offerPacket(packet);
                }

            }

        }
        
    }
    
    
    private static class ServerDatagramHandler extends Thread{

        private AWSRobotServer server;
        private String fromClient;
        
        public ServerDatagramHandler(AWSRobotServer server){
            this.server = server;
            fromClient = "";
        }
        
        public void run(){
            
            while(!Thread.interrupted()){
                try {
                    DatagramPacket packet = server.getPacket();
                    byte[] dataReceived = packet.getData();
                    fromClient = new String(dataReceived).trim();

                    //System.out.println("Data received: " + fromClient);

                    // remove the "~" at the start of the message that 
                    // denotes a message for the server
                    fromClient = fromClient.split("~", 2)[1]; 

                    String[] messageParts = fromClient.split(">>", 2);
                    
                    String sender = messageParts[0];
                    
                    if (sender.equals("robot")){
                        
                        System.out.println("received UDP from robot: " + messageParts[1]);
                        
                        server.sendUDPMessageToCustomers(fromClient);
                        
                    }else{
                        // shouldn't happen, customers/clients don't send UDPs
                        System.out.println("received UDP from customer: " + messageParts[1]);
                        
                        server.handleCustomerMessage(sender, messageParts[1]);
                        
                    }
                } catch (InterruptedException e) {
                    System.out.println(e.getMessage());
                }
            }
        }   
    }
    
    private static class ConnectedClient extends Thread{
        
        public AWSRobotServer server;
        public BufferedReader br;
        public BufferedOutputStream os;
        public Socket clientSocket;
        
        public BlockingQueue<String> tcpInQueue;
        
        private InetAddress ip;
        
        private String id = null;
        
        public ConnectedClient(Socket toClient, AWSRobotServer server, 
            BufferedOutputStream os, BufferedReader br){
        
            this.server = server;
            this.br = br;
            this.os = os;
            this.clientSocket = toClient;
            
            ip = toClient.getInetAddress();
            tcpInQueue = new LinkedBlockingQueue<>();
        
        }


        public void run() {
            
            String fromUser = null;
            try {
                
                while (id == null) {

                    fromUser = br.readLine();
                    
                    System.out.println("Received from user m: " + fromUser);
                    
                    if (fromUser.matches(".*id:.*")) {
                        id = fromUser.split("id:")[1];
                    }
                }
                
                System.out.println("New client with id: " + id);
                
                if(id.equals("robot")){
                    System.out.println("adding robot to server");
                    server.setRobot(this);
                }else{
                    System.out.println("adding new customer to queue");
                    server.addCustomer(this);
                }
                
                String stringFromUser;
                
                while(!Thread.interrupted()){
                    try {
                        stringFromUser = br.readLine();
                        if(stringFromUser != null) {
                        	tcpInQueue.add(stringFromUser);
                        	//System.out.println(stringFromUser);
                        }else {
                        	System.out.println("Null read in, client disconnected");
                        	break;
                        }
                    } catch (IOException e) {
                        System.out.println("Exception in Connected User: " + e.getMessage());
                        break;
                    }
                }
                
            } catch (IOException e) {
                System.err.println("ConnectedClient: IOException: " + e.getMessage());
            } 
            
            if(id.equals("robot")){
            	System.out.println("removing robot");
                server.removeRobot();
            }else{
            	System.out.println("removing customer with id:" + id);
                server.removeCustomer(this);
            }
            
            try{
                clientSocket.close();
            }catch(Exception e){
                // exit
            }
            
            
        }
        
        public void sendTCP(String message){
            
            try{
                os.write((message+"\n").getBytes());
                os.flush();            
            }catch(IOException e) {
                System.out.println("Error sending TCP to " + id + " @ " + ip.getHostAddress() );
            }
            
        }
        
        public void sendUDP(String message){
        	//System.out.println("sending UDP to " + id);
            byte[] toSend = message.getBytes();
            DatagramPacket dp = new DatagramPacket(toSend, toSend.length,ip, APP_UDP_PORT );
            try {
                server.getDatagramSocket().send(dp);
                //System.out.println("sent udp to " + id + "@ " + ip);
            } catch (IOException e) {
                System.out.println("Error sending packet to " + id + " @ " + ip.getHostAddress() );
            }

        }
        
    }
    
    private static class ServerWorker extends Thread{
        
        AWSRobotServer server;
        
        
        public ServerWorker(AWSRobotServer s){
            this.server = s;
        }
        
        public void run(){
            
            System.out.println("Starting loop");
            
            boolean robotConnectReport = false;
            boolean robotDisconectReport = false;
            
            while(!Thread.interrupted()){
                
                
                if(server.getRobot() != null){
                    
                    if(!robotConnectReport){
                        System.out.println("robot found: informing customers");
                        robotConnectReport = true;
                        robotDisconectReport = false;
                    }
                    
                    
                    server.sendUDPMessageToCustomers("ROBOT_LIVE");
                    
                    String robotTCP = server.getRobot().tcpInQueue.poll();
                    
                    if(robotTCP != null){
                        server.sendTCPMessageToCustomers(robotTCP);
                    }
                    
                    processClientTCPs();
                    
                }else{
                    
                    if(!robotDisconectReport){
                        System.out.println("no robot: sending udp to inform customers");
                        robotDisconectReport = true;
                        robotConnectReport = false;
                    }
                    
                    server.sendUDPMessageToCustomers("NO_ROBOT");
                    
                    processClientTCPs();
                    
                }
                
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    System.out.println("Error, server worker interrupted: " + e.getMessage());
                }
                
            }
            
            
        }

		private void processClientTCPs() {
			String messageFromCustomer;
			Iterator<ConnectedClient> i = server.getCustomers().iterator();
			messageFromCustomer = null;
			
			while(i.hasNext()){
			    
			    ConnectedClient c = i.next();
			    //System.out.println("processing customer " + c.id);
			    
			    messageFromCustomer = c.tcpInQueue.poll();
			    
			    while(messageFromCustomer != null){
			        
			    	if(messageFromCustomer.contains("id:")) {
			        	System.out.println("received connection request from customer: " + messageFromCustomer);
			        	c.sendTCP("server>>connection_accepted");
			        }else {
			        	//System.out.println("received message from customer: " + messageFromCustomer);
				        
				        if(server.getRobot() != null) server.getRobot().sendTCP(messageFromCustomer);
			        }
			        messageFromCustomer = c.tcpInQueue.poll();
			        
			    }
			    
			}
		}
        
    }
    
    
}
