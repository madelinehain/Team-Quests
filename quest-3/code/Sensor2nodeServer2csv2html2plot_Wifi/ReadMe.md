# Sensor -> Node Server -> CSV File -> HTML -> Plot

## Node Server
"index_wifi.js" is the node server which reads from port 3333 in wifi, converts the serial data into a csv file, creates an array from that csv file, and passes the array to the html file. 

## Plot
"index_wifi_button.html" is the html file, which takes the data array from the server and formats it into an updating plot.