// MODULES
// Wifi
const dgram = require('dgram');
// Port and IP
const PORT_CLIENT = 3333;        // Port where the thermistor data comes from
const HOST = '192.168.1.6';   // Server Host Address (whatever is running this node server)
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
// let avg = 0;
let avg_transit_time = 0;
const checkpoint_num = 10;

// let qrdata;
let checkpoint = 0;

var server_send_message = 0;

let old_QR = "NOT START";

// NONE -> before rally has started, START -> rally started, END -> end of rally
var start_end_state = 'NONE';

// Handle incoming messages from the UDP socket
server.on('message', function (message, remote) {
  console.log(remote.address + ':' + remote.port +' - ' + message);
  counter++;

  console.log('Remote Address: ' + remote.address);
  
  // Check if the message is from the QR Code Reader
  if (remote.address == '192.168.1.33') {
    // // Switch case for QR data color
    // qrdata = message.slice(14, 15);
    // // switch (qrdata) {
    // //   case "R":
    // //     checkpoint = 1;
    // //     break;
    // //   case "S":
    // //     checkpoint = 2;
    // //     break;
    // //   case "B":
    // //     checkpoint = 3;
    // //     break;
    // //   case "G":
    // //     checkpoint = 4;
    // //     break;
    // //   default:
    // //     checkpoint = 0;
    // // }
    
    // Calculate rolling average
    // avg = (avg * counter + transit_time) / (counter + 1);
    // counter++;
    
    // Convert message to string and append rolling average and time difference
    const mes_str = message.toString();

    const QR_message = mes_str.split(/[ ,]+/);


    // var current_timestamp = QR_message[1];
    // var a = current_timestamp.split(':'); // split it at the colons
    // // minutes are worth 60 seconds. Hours are worth 60 minutes.
    // var current_seconds = (+a[0]) * 60 * 60 + (+a[1]) * 60 + (+a[2]); 
    // var transit_time = current_seconds - prev_seconds;

    // // If it is the 1st message, the average transit time is the difference.
    // if (counter == 1){
    //     avg_transit_time = transit_time;
    // }


    // avg = ((avg * counter) + transit_time) / (counter);
    // avg_transit_time = ((avg_transit_time * counter) + transit_time) / (counter);
    
    // // Insert QR_message into collection
    // const messageObj = {date: QR_message[0], time: QR_message[1], qr: QR_message[2], checkpoint, avg, transit_time, avg_transit_time, counter};
    // collection.insert(messageObj, (error, result) => {
    //   if (error) {
    //     console.error(error);
    //   } else {
    //     console.log(result);
    //   }
    // });
    

    // If next checkpoint was reached
    if (QR_message[2] != old_QR) {
      // if (start_end_state != 'START')
      console.log("NEW QR CODE ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !");
      checkpoint ++; // increment checkpoint index

      var current_timestamp = QR_message[1];
      var a = current_timestamp.split(':'); // split it at the colons
      // minutes are worth 60 seconds. Hours are worth 60 minutes.
      var current_seconds = (+a[0]) * 60 * 60 + (+a[1]) * 60 + (+a[2]);
      var transit_time = current_seconds - prev_seconds;

      var lap_time = current_seconds - start_seconds;

      // // If it is the 1st message, the average transit time is the difference.
      // if (counter == 1){
      //   avg_transit_time = transit_time;
      //   console.log("OKOKOKOKOKOKOKOKOK!!!");
      // }

      // Calculate Average Transit Time
      avg_transit_time = ((avg_transit_time * counter) + transit_time) / (counter);

      // Insert QR_message into collection
      const messageObj = {date: QR_message[0], time: QR_message[1], qr: QR_message[2], checkpoint, transit_time, avg_transit_time, counter, lap_time};
      collection.insert(messageObj, (error, result) => {
        if (error) {
          console.error(error);
        } else {
          console.log(result);
        }
      });

      // Send Average Transit Time & Checkpoint QR Time to HTML Page

    }
    else if (QR_message[2] == "START") {
      checkpoint = 0;

      var a = QR_message[1].split(':'); // split it at the colons
      // minutes are worth 60 seconds. Hours are worth 60 minutes.
      var start_seconds = (+a[0]) * 60 * 60 + (+a[1]) * 60 + (+a[2]); ;

      // Set Start Message for HTML Page
      start_end_state = 'START';

      console.log("Started Rally!");
    }
    else if (QR_message[2] == "END") {
      // Set ending message for HTML Page
      start_end_state = 'END';

      console.log("Ended Rally!");
    }


    console.log("Old: " + old_QR + ", New: " + QR_message[2]);
    // Update Old QR Code Message
    old_QR = QR_message[2];


    // Update previous Average Transit Time (seconds)
    prev_seconds = current_seconds;
  }
  // Check if the message is from the ESP32 Alphanumeric Display
  else if (remote.address == '192.168.1.25') {
    console.log('Message from ESP32: ' + message.toString());

    // Get the Message we will send to the Alphanumeric Display
    server_send_message = avg_transit_time.toString();

    // Send it
    server.send(server_send_message, remote.port, remote.address, function(error){
      if(error){
        console.log('Server Send Error!');
      }
      // If no error: Send Message to Client(s)
      else{
        console.log('Sent: ' + server_send_message);
      }
    });
  }

});


// request data at http://localhost:8080/data or just "/data"
app.get('/start_end_state', function(req, res) {
  res.send(start_end_state);  // Send array of data back to requestor
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

// viewed at http://localhost:8080
app.get('/', function(req, res) {
  res.sendFile(path.join(__dirname + '/Q6_browser_v2.html'));
});

// Start the Server
app.listen(PORT, () => {
    console.log(`Server is running on port ${PORT}`);
});
