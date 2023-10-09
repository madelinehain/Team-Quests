/*
  Maxwell Bakalos, Miguel Ianus-Valdiva, Emre Karabay, Madeline Hain
  EC444 Smart & Connected Systems
  Quest 3 - Cat Tracker
  UDP Server for Thermistor, Video, & LED Button
  3/31/2023
*/

// This code takes a CSV file and creates an array which it passes to an html file for plotting
// in the webpage of the "NodeServerHostIP:8080" (port 8080)

// Required module
var dgram = require('dgram');
// Port and IP
var PORT = 3333;          // Port where the thermistor data comes from
var HOST = '192.168.1.6'; // Server Host Address (whatever is running this node server)
// Create socket
var server = dgram.createSocket('udp4');

// Read CSV File & Send Array to HTML File
var express = require('express');
var app = express();
var path = require('path');
var csv = require("csv-parse");
// Create CSV File
var fs = require('fs');
// Button for LED
const bodyParser = require('body-parser');

var server_send_message = "Ok!"   // Message Sent to ESP32
var button_toggle = 0             // LED Button Toggle

// Listen for Button Pressed Signal from HTML Page //////////
app.use(bodyParser.urlencoded({ extended: true })); 
// Receive LED Toggle Signal
app.post('/button', (req, res) => {
  // res.send('${req.body.LED_toggle}');

  console.log("BUTTON PRESSED!!!")
  if (button_toggle == 1) {
    server_send_message = "Ok!"
    button_toggle = 0 // set button to OFF
  }
  else if (button_toggle == 0) {
    server_send_message = "BUTTON_PRESSED"
    button_toggle = 1 // set button to ON
  }
});

// const port = 8080;

// app.listen(port, () => {
//   console.log(`Server running on port${port}`);
// });


// Create server that listens on a port
server.on('listening', function () {
  var address = server.address();
  console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// Create CSV File
fs.open('Sensor_Output.csv', 'w', function (err, file) {
  if (err) throw err;
  console.log('Saved File!');
}); 

let data_str = ""
// On connection, print out received message
server.on('message', function (message, remote) {
  console.log(remote.address + ':' + remote.port +' - ' + message);

  // data_str =  message + "\n"
  data_str = message
  // Append line to CSV file
  fs.appendFile('Sensor_Output.csv', data_str, 'utf8', function(error) {
    // In case of a error throw err.
    if (error) throw error;
    console.log('File Appended!');
  })

    // Send Ok acknowledgement
    server.send(server_send_message,remote.port,remote.address,function(error){
      if(error){
        console.log('MEH!');
      }
      else{
        console.log('Sent: ' + server_send_message);
      }
    });

});

// Bind server to port and IP
server.bind(PORT, HOST);

// viewed at http://localhost:8080
app.get('/', function(req, res) {
  res.sendFile(path.join(__dirname + '/Q3_Web_Observation_Portal.html'));
});

// request data at http://localhost:8080/data or just "/data"
app.get('/data', function(req, res) {
  var data = [];  // Array to hold all csv data
  fs.createReadStream('Sensor_Output.csv')
  .pipe(csv.parse())      // fixed by using csv.parse() instead of csv()
  .on('data', (row) => {
    console.log(row);
    data.push(row);  // Add row of data to array
  })
  .on('end', () => {
    console.log('CSV file successfully processed');
    res.send(data);  // Send array of data back to requestor
  });
});
app.listen(8080);
