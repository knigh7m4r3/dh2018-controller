const WebSocket = require('ws')
var pre;
var clientid = 0;
var first = true;
const wss = new WebSocket.Server({port : 2000 } );

wss.on('connection', function connection(ws) {
    
  var msg = new Object();
  msg.type="info";
  msg.id=clientid++;

  var JSONmsg = JSON.stringify(msg);
  ws.send(JSONmsg);

  ws.on('message', function incoming (message) {
    console.log('received:%s', message);
    if ( message == "presenter"){
      console.log(message);
      console.log("Presenter aktivated!");
      pre=ws;
			var createMsg = new Object();
			createMsg.type="create";
			createMsg.id="1";
			var createJSON = JSON.stringify(createMsg);
			pre.send(createJSON);
    }else if ( message.type == "create" && pre != null) {
      console.log("New client!");
      pre.send(message);
    }else {
			if(pre != null){
	      console.log("Client update!");
	      pre.send(message);
			}
   }
  });
});

wss.on('error', function error(){
  console.log('fehler');  
});

