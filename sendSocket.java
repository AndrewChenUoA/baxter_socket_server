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