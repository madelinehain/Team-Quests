/*
  Maxwell Bakalos, Miguel Ianus-Valdiva, Emre Karabay, Madeline Hain
  EC444 Smart & Connected Systems
  Quest 3 - Cat Tracker
  UDP Server for Thermistor, Video, & LED Button
  4/30/2023
*/

const dgram = require('dgram');
var fs = require('fs');

const HOST = '192.168.1.6';
const PORT = 30000;
const client = dgram.createSocket('udp4');
var csv = require("csv-parse");

var QR_data = [];  // Array to hold all csv data
var SID_message = 'test1, test2';

let send_interval = 2000;   // interval to send SID in milliseconds

///// WORKS
function send_SID() {
    // Read CSV File Into an Array
    fs.createReadStream('barcodes.csv')
    .pipe(csv.parse())      // fixed by using csv.parse() instead of csv()
    .on('data', (row) => {
        console.log(row);
        QR_data.push(row);  // Add row of data to array
    })

    .on('end', () => {
        console.log('CSV file successfully processed');
        console.log(QR_data.length)
        console.log(QR_data)
        if (QR_data.length > 0){ //&& QR_data[QR_data.length - 1] !== undefined) {
            console.log('HELLO TEST');
            SID_message = QR_data[QR_data.length - 1].join(', ');
            console.log(SID_message)
        }
        const message = new Buffer(SID_message);

        client.send(message, 0, message.length, PORT, HOST, function(err, bytes) {
            if (err) throw err;
            console.log('UDP message sent to ' + HOST +':'+ PORT);
            client.close();
        });

    });
}
setInterval(send_SID, send_interval);
