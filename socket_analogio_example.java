import java.net.*;
import java.io.*;

public class socket_analogio {
    public static void main (String[] args) {
        //Set the io_component value, with default value
	    String io_component = new String("torso_fan");
        if (args.length > 0) {
            io_component = args[0];
        }

        String name = io_component;
        System.out.println("Object Name: " + name);
        System.out.println(sendSocket("analog_state "+name));
    
        while(true) {
            //Ramp Up
            for (int i=0;i<=100;i++){
                sendSocket("analog_setoutput "+name+" "+Integer.toString(i));
                System.out.println(i);
                try{ //Wait for 0.1 seconds
                    Thread.sleep(100);
                } catch(InterruptedException ex) {
                    System.out.println(ex);
                }
            }
            
            try{ //Wait for 2 seconds
                Thread.sleep(2000);
            } catch(InterruptedException ex) {
                System.out.println(ex);
            }
            
            //Ramp Down
            for (int i=100;i>=0;i--){
                sendSocket("analog_setoutput "+name+" "+Integer.toString(i));
                System.out.println(i);
                try{ //Wait for 0.1 seconds
                    Thread.sleep(100);
                } catch(InterruptedException ex) {
                    System.out.println(ex);
                }
            }

            try{ //Wait for 2 seconds
                Thread.sleep(2000);
            } catch(InterruptedException ex) {
                System.out.println(ex);
            }
        }    
    }
    
    public static String sendSocket(String command) {
        String str_out = new String("");
        try {
            Socket client = new Socket("192.168.1.114", 10111);            
            BufferedWriter out = new BufferedWriter(new OutputStreamWriter(client.getOutputStream()));
            out.write(command+"\n");
            out.flush();
            BufferedReader in = new BufferedReader(new InputStreamReader(client.getInputStream()));
            str_out = in.readLine();
            in.close();
            out.close();
            client.close();            
        } 
        catch (MalformedURLException e) {} 
        catch (IOException e) {}
        return str_out;
    }
}

