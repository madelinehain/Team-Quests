/*
  Maxwell Bakalos
  EC444 Smart & Connected Systems
  Quest 2 - Skill 16
  Node JS
  2/24/2023
*/

// Import the Serial Port Modules
const { SerialPort, ReadlineParser } = require('serialport')

// Create a Port (COM7 for ESP32)
const port = new SerialPort({path: 'COM7', baudRate: 115200,})
// Create a Parser
const parser = port.pipe(new ReadlineParser())

// Print to Console (see in terminal)
parser.on('data', console.log)
