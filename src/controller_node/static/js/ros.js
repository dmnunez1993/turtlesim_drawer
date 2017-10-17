var Position = function () {
    this.X = 0;
    this.Y = 0;
}

Position.prototype.setPosition = function(x, y) {
    this.x = x;
    this.y = y;
}

var canvas = null;
var drawer = null;
var turtle_position = null;



$( document ).ready(function() {
    // Connecting to ROS
    // -----------------
    var ros = new ROSLIB.Ros();

    // If there is an error on the backend, an 'error' emit will be emitted.
    ros.on('error', function(error) {
        $connectionStatus = $('#connectionStatus');
        $connectionStatus.html('An error occurred!!');
        $connectionStatus.css('color', 'red');
    });

    // Find out exactly when we made a connection.
    ros.on('connection', function() {
        console.log('Connection established');
        $connectionStatus = $('#connectionStatus');
        $connectionStatus.html('Connection established');
        $connectionStatus.css('color', 'green');
        $('#startStopButton').prop('disabled', false);

    });

    ros.on('close', function() {
        $connectionStatus = $('#connectionStatus');
        $connectionStatus.html('Connection closed!!');
        $connectionStatus.css('color', 'red');
    });

    // Create a connection to the rosbridge WebSocket server.
    ros.connect('ws://localhost:9090');

    var poseTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/turtle1/pose',
        messageType: 'turtlesim/Pose'
    });
      // Subscribes to the robot's pose. When rosbridge receives the pose
      // message from ROS, it forwards the message to roslibjs, which calls this
      // callback.

    var pause = false;
    var started = false;
    var start_requested = true;

    poseTopic.subscribe(function(message) {
      if (drawer) {
          w = drawer.canvas.width;
          h = drawer.canvas.height;
          turtle_x = ((message.x/12)*w);
          turtle_y = h - ((message.y/12)*h);
          turtle_position = new Position();
          turtle_position.setPosition(turtle_x, turtle_y);
          drawer.gotoxy(turtle_position.x, turtle_position.y);
          drawer.draw();

      } else {
          canvas = document.getElementById('turtleFollower');
          w = canvas.width;
          h = canvas.height;
          turtle_x = ((message.x/12)*w);
          turtle_y = h - ((message.y/12)*h);
          turtle_position = new Position();
          turtle_position.setPosition(turtle_x, turtle_y);
          drawer = new Drawer();
          drawer.init(canvas, turtle_position.x, turtle_position.y);
          drawer.gotoxy(turtle_position.x, turtle_position.y);
          drawer.draw();
      }

    });

    var statTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/turtlesim_drawer/stat',
        messageType: 'turtlesim_drawer/Stat'
    });

    statTopic.subscribe(function(message)   {
        pause = message.paused;
        started = message.started;
        if (!pause) {
            $('#pauseContinueButton').html('Pause');
        }   else {
            $('#pauseContinueButton').html('Continue');
        }

        if (!started) {
            if (!start_requested)   {
                $('#startStopButton').html('Start');
            }   else {
                if (drawer) {
                    drawer.erase();
                }
            }

            $('#pauseContinueButton').prop('disabled', true);

        }   else {
            $('#startStopButton').prop('disabled', false);
            $('#startStopButton').html('Stop');
            $('#pauseContinueButton').prop('disabled', false);
            start_requested = false;
        }
        if (started)  {
            $('#shapeSelect').prop('disabled', true);
        }   else {
            $('#shapeSelect').prop('disabled', false);
        }
    });

    $('#pauseContinueButton').click(function () {
        if (!pause)    {
            var pauseClient = new ROSLIB.Service({
                ros : ros,
                name : '/turtlesim_drawer/pause',
                serviceType : 'std_srvs/Empty'
            });

            var request = new ROSLIB.ServiceRequest();

            pauseClient.callService(request, function(result) {

            });

        }   else {
            var resumeClient = new ROSLIB.Service({
                ros : ros,
                name : '/turtlesim_drawer/resume',
                serviceType : 'std_srvs/Empty'
            });

            var request = new ROSLIB.ServiceRequest();

            resumeClient.callService(request, function(result) {

            });
        }
    });

    $('#startStopButton').click(function () {
        if (!started)    {
            var startClient = new ROSLIB.Service({
                ros : ros,
                name : '/turtlesim_drawer/start',
                serviceType : 'std_srvs/Empty'
            });

            var request = new ROSLIB.ServiceRequest();

            startClient.callService(request, function(result) {
                drawer.erase();
                drawer = null;
            });

            start_requested = true;
            $( this ).prop('disabled', true);
            $( this ).html('Starting...');
        }   else {
            var stopClient = new ROSLIB.Service({
                ros : ros,
                name : '/turtlesim_drawer/stop',
                serviceType : 'std_srvs/Empty'
            });

            var request = new ROSLIB.ServiceRequest();

            stopClient.callService(request, function(result) {

            });
        }
    });

    $('#shapeSelect').change(function ()    {
        var shapeClient = new ROSLIB.Service({
            ros : ros,
            name : 'turtlesim_drawer/shape',
            serviceType : 'turtlesim_drawer/Shape'
        });

        var request = new ROSLIB.ServiceRequest({
            shape : $('#shapeSelect').val()
        });


        shapeClient.callService(request, function(result) {
            console.log(result.current_shape);
        });
    });
});
