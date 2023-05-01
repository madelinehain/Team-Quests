/*
  Maxwell Bakalos, Miguel Ianus-Valdiva, Emre Karabay, Madeline Hain
  EC444 Smart & Connected Systems
  Quest 3 - Cat Tracker
  UDP Server for Thermistor, Video, & LED Button
  4/30/2023
*/
// UDP Client
var PORT = 33333;
var HOST = '192.168.1.6';
var dgram = require('dgram');
var client = dgram.createSocket('udp4');
// Read CSV File
var fs = require('fs');
var csv = require("csv-parse");
// Send QR Code Scooter ID (SID)
const send_QRcode_interval = 2000; // interval to send QR code data in miliseconds

var SID_message = 'test1, test2';

// On connection, send QR Code Data to Database Server
setInterval(() => {

    var QR_data = [];  // Array to hold all csv data

    // Read CSV File Into an Array
    fs.createReadStream('QR_Code_Data.csv')
    .pipe(csv.parse())      // fixed by using csv.parse() instead of csv()
    .on('data', (row) => {
        console.log(row);
        QR_data.push(row);  // Add row of data to array
    })
    .on('end', () => {
        console.log('CSV file successfully processed');
        // res.send(QR_data);  // Send array of data back to requestor
    });

    // var SID_message = QR_data[QR_data.length - 1].join('');

    // var SID_message = '';
    if (QR_data.length > 0 && QR_data[QR_data.length - 1] !== undefined) {
        SID_message = QR_data[QR_data.length - 1].join('');
    }


    client.send(SID_message, PORT, HOST, function(err) {
    if (err) throw err;
    console.log('UDP message (SID) sent to ' + HOST +':'+ PORT);
    client.close();
    });
}, send_QRcode_interval);