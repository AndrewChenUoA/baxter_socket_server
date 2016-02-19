import java.net.*;
import java.io.*;

public class socket_digitalio {
    public static void main (String[] args) {
        //Set the io_component value, with default value
	    String io_component = new String("right_itb_light_outer");
        if (args.length > 0) {
            io_component = args[0];
        }

        String name = io_component;
        System.out.println("Object Name: " + name);

        System.out.println("Initial state: " + sendSocket("digital_state "+name));
    
        while(true) {
            // turn on light
            sendSocket("digital_setoutput "+name+" True");
            try{ //Wait for 1 second
                Thread.sleep(1000);
            } catch(InterruptedException ex) {
                System.out.println(ex);
            }
            System.out.println("New state: " + sendSocket("digital_state "+name));

            //turn off light
            sendSocket("digital_setoutput "+name+" False");
            try{ //Wait for 1 second
                Thread.sleep(1000);
            } catch(InterruptedException ex) {
                System.out.println(ex);
            }
            System.out.println("New state: " + sendSocket("digital_state "+name));
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

