// MODULES
// Wifi
const dgram = require('dgram');
// Port and IP
const PORT_CLIENT = 3333;        // Port where the thermistor data comes from
const HOST = '192.168.1.34';   // Server Host Address (whatever is running this node server)
const PORT = process.env.PORT || 8080;  // Port where server is running from "NodeServerHostIP:PORT"
// Create socket
const server = dgram.createSocket('udp4');
// Communicate with HTML File
const express = require('express');
const app = express();

const { Db } = require('tingodb')({ memStore: true });
const db = new Db('./mydb', {});
const collection = db.collection('scooterCollection');

let counter = 1;
// let prev_timestamp_num = 0;
// let prev_timestamp= 0;
let prev_seconds = 0;
let avg = 0;
let avg_diff = 0;
const checkpoint_num = 10;
// Handle incoming messages from the UDP socket
server.on('message', function (message, remote) {
    console.log(remote.address + ':' + remote.port +' - ' + message);
    counter++;
    // Extract timestamp from message using regular expression
    // const timestamp_str = message.toString().match(/^\d+\.\d+/)[0];
    // const timestamp_num = parseFloat(timestamp_str);
    
    // Calculate time difference between current and previous timestamps
    // const diff = timestamp_num - prev_timestamp_num;
    
    // Switch case for QR data color
    let qrdata = message.slice(14, 15);
    let checkpoint;
    switch (qrdata) {
      case "R":
        checkpoint = 1;
        break;
      case "S":
        checkpoint = 2;
        break;
      case "B":
        checkpoint = 3;
        break;
      case "G":
        checkpoint = 4;
        break;
      default:
        checkpoint = 0;
    }
    
    // Calculate rolling average
    // avg = (avg * counter + diff) / (counter + 1);
    // counter++;
    
    // Convert message to string and append rolling average and time difference
    const mes_str = message.toString();
  
    const message_SID_FID = mes_str.split(/[ ,]+/);
    var current_timestamp = message_SID_FID[1];
    var a = current_timestamp.split(':'); // split it at the colons
    // minutes are worth 60 seconds. Hours are worth 60 minutes.
    var current_seconds = (+a[0]) * 60 * 60 + (+a[1]) * 60 + (+a[2]); 
    var diff = current_seconds - prev_seconds;

    if (counter == 1){
        avg_diff = diff;
    }

    avg = ((avg * counter) + diff) / (counter);
    avg_diff = ((avg_diff * counter) + diff) / (counter);
    
    // Insert message_SID_FID into collection
    const messageObj = {date: message_SID_FID[0], time: message_SID_FID[1], qr: message_SID_FID[2], checkpoint, avg, diff, avg_diff, counter};
    collection.insert(messageObj, (error, result) => {
      if (error) {
        console.error(error);
      } else {
        console.log(result);
      }
    });
    
    // Update previous timestamp number
    // prev_timestamp_num = timestamp_num;
    // prev_timestamp = current_timestamp;
    prev_seconds = current_seconds;
    let server_send_message = avg_diff.toString();
    server.send(server_send_message,remote.port,remote.address,function(error){
      if(error){
        console.log('Server Send Error!');
      }
      // If no error: Send Message to CLient(s)
      else{
        console.log('Sent: ' + server_send_message);
        // server_send_message = "Ok!"
      }
    });
  });  

server.on('error', (err) => {
  console.log(`server error:\n${err.stack}`);
  server.close();
});

// Create server that listens on a port
server.on('listening', function () {
    const address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// Bind server to port and IP
server.bind({
    address: HOST,
    port: PORT_CLIENT,
    // exclusive: true
});

// Start the Server
app.listen(PORT, () => {
    console.log(`Server is running on port ${PORT}`);
});
