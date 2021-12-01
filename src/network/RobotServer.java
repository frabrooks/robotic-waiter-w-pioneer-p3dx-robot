import java.net.*;	
import java.io.*;

public class RobotServer {

    public static void main(String[] args) {
        
        ServerSocket serverSocket = null;
        
        int portNumber = 1776;
        
        try {
            serverSocket = new ServerSocket(portNumber);
        } catch (IOException e) {
            System.out.println("Couldn't listen on port " + portNumber);
        }
        
        while (true) {
            try {
                // We loop for ever, as servers usually do.
                while (true) {
                    // Listen to the socket, accepting connections from new
                    // clients:
                    Socket socket = serverSocket.accept();

                    // Create input and output streams
                    BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                    PrintWriter out = new PrintWriter(socket.getOutputStream(), true);

                    System.out.println("Someone has connected");
                    
                    String str = "";
                    
                    while ((str = in.readLine()) != null){
                        
                       System.out.println(str);
                       out.println("Hello");
                       
                       if(str.equals("bye")){
                           
                            break;
                       }
                        
                    }
                    
                    in.close();
                    
                    out.close();
                    
                    socket.close();
                    
                    socket = null;
                    

                }
            } catch (IOException e) {
                System.out.println("IO error " + e.getMessage());

            } finally {
                try {
                    serverSocket.close();
                } catch (IOException e) {
                    // Nothing to do
                }
            }
        }
        
		
	}

    


}