/*
  Maxwell Bakalos, Miguel Ianus-Valdiva, Emre Karabay, Madeline Hain
  EC444 Smart & Connected Systems
  Quest 5 - NFC Scooter Key Fob
  UDP Server for Thermistor, Video, & LED Button
  5/2/2023
*/

// MODULES
// Wifi
var dgram = require('dgram');
// Port and IP
var PORT_CLIENT = 3333;        // Port where the thermistor data comes from
var HOST = '192.168.1.6';   // Server Host Address (whatever is running this node server)
const PORT = process.env.PORT || 8080;  // Port where server is running from "NodeServerHostIP:PORT"
// Create socket
var server = dgram.createSocket('udp4');
// Communicate with HTML File
var express = require('express');
var app = express();

const { Db } = require('tingodb')({ memStore: true });
const db = new Db('./mydb', {});
const collection = db.collection('scooterCollection');


// Put Scooter ID -Fob ID Pair in Database
var scooterID_fobID = {A: "SID01", B: "FID01"};
collection.insert(scooterID_fobID, (error, result) => {
    if (error) {
      console.error(error);
    } else {
      console.log(result);
    }
});


// randomly generated N = 10 length array 0 <= A[N] <= 9
var random_array = Array.from({length: 8}, () => Math.floor(Math.random() * 9));
let server_send_message = random_array.join('');

var valid_SID_FID_pair_counter = 0;

// Handle incoming messages from the UDP socket
server.on('message', function (message, remote) {
  console.log(remote.address + ':' + remote.port +' - ' + message);
  
  // turn message into string
  let mes_str = message.toString();

  // Split Message from string to array: "SID, FID" --> ["SID", "FID"]
  var message_SID_FID  = mes_str.split(/[ ,]+/);

//   console.log(message_SID_FID[0]); // debug received message

  // Find the first document that matches the message
  collection.findOne({ A: message_SID_FID[2], B: message_SID_FID[3] }, function(err, doc) {
    if (err) {
      console.log(err);
    } 
    else {
      if (doc) {
        valid_SID_FID_pair_counter = 3; // set valid pair count to 3

        // The message is already in the database
        console.log('Message already in database:', doc);

        // // Send Out Random Key to Clients
        // server.send(server_send_message,remote.port,remote.address,function(error){
        //   if(error){
        //     console.log('Server Send Error!');
        //   }
        //   // If no error: Send Message to CLient(s)
        //   else{
        //     console.log('Sent: ' + server_send_message);
        //     // server_send_message = "Ok!"
        //   }
        // });
      } 
      else {
        // The message is not in the database yet
        console.log('Message not in database:', message);

        valid_SID_FID_pair_counter += -1; // decrease valid pair count by 1
      }

      // If there was a recent valid SID-FID pair . . .
      if (valid_SID_FID_pair_counter > 0) {

        // Send Out Random Key to Clients
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
      }
      else {
        // Send Out "No Key" to Clients
        server.send("No Key",remote.port,remote.address,function(error){
          if(error){
            console.log('Server Send Error!');
          }
          // If no error: Send Message to CLient(s)
          else{
            console.log('Sent: ' + "No Key");
            // server_send_message = "Ok!"
          }
        });
      }


    }
  });

});

server.on('error', (err) => {
  console.log(`server error:\n${err.stack}`);
  server.close();
});

// Create server that listens on a port
server.on('listening', function () {
    var address = server.address();
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