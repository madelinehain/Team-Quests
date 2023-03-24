// Required module
var dgram = require('dgram');

// Port and IP
var PORT = 3333;           // Port
var HOST = '192.168.1.40'; // Server Host Address (whatever is running this node server)

var client = dgram.createSocket('udp4');
var message = 'Hello ESP32!'

client.on('listening', function () {
    var address = client.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

client.on('message', function (message, remote) {

    console.log(remote.address + ':' + remote.port +' - ' + message);

});

for (let i = 1; i <= 5; i++) {
  client.send(message, 0, message.length, PORT, HOST, function(err, bytes) {

      if (err) throw err;
      console.log('UDP message sent to ' + HOST +':'+ PORT);

  });
}


// const serverPort = 8080; // Change this to the port your ESP32 server is listening on
// const serverAddress = '192.168.1.100'; // Change this to the IP address of your ESP32 server

// client.on('message', (msg, rinfo) => {
//   console.log(client received: ${msg} from ${rinfo.address}:${rinfo.port});
// });

// for (let i = 1; i <= 3; i++) {
//   const message = Message ${i} to ESP32;
//   client.send(message, serverPort, serverAddress, (err) => {
//     if (err) {
//       console.log(client error: ${err});
//     } else {
//       console.log(client sent message ${i} to ESP32);
//     }
//   });
// }