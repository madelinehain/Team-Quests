// Required module
var dgram = require('dgram');

// Port and IP
var PORT = 3333;
// var HOST = '192.168.1.105';
// var HOST = '192.168.1.1';
// var HOST = '192.168.1.38';
var HOST = '192.168.1.29'; // Server Host Address (whatever is running this node server)

// Create socket
var server = dgram.createSocket('udp4');

// Create server that listens on a port
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// On connection, print out received message
server.on('message', function (message, remote) {
    console.log(remote.address + ':' + remote.port +' - ' + message);

    // Send Ok acknowledgement
    server.send("Ok!",remote.port,remote.address,function(error){
      if(error){
        console.log('MEH!');
      }
      else{
        console.log('Sent: Ok!');
      }
    });

});

// Bind server to port and IP
server.bind(PORT, HOST);