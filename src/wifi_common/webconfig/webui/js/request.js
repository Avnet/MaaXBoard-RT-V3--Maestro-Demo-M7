
/* Make request to server */
function make_request(url)
{
	var http_request = false;

	data_received = 1;

    // // Mozilla, Safari,...
    // if (window.XMLHttpRequest)
    // {
    // 	http_request = new XMLHttpRequest();
    // 	if (http_request.overrideMimeType)
    //     {
    //     	http_request.overrideMimeType('text/xml');
    // 	}
    // }
    // // IE
	// else if (window.ActiveXObject)
    // { 
 	// 	try
    //     {
    //         http_request = new ActiveXObject("Msxml2.XMLHTTP");
    //     }
	// 	catch (e)
    //     {
	// 		try
    //         {
    //             http_request = new ActiveXObject("Microsoft.XMLHTTP");
    //         }
	// 		catch (e)
    //         {
    //         }
    //     }
    // }

    // if (!http_request)
    // {
    // 	alert('Giving up :( Cannot create an XMLHTTP instance');
    //     return false;
    // }

    http_request = new XMLHttpRequest();
    http_request.onreadystatechange = function() { alertContents(http_request); };
    http_request.open('GET', url, true);
    http_request.send(null);
}

/* Make request to server */
function make_post_request(url, params)
{
    var http = new XMLHttpRequest();
    http.open('POST', url, true);

    //Send the proper header information along with the request
    // http.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
    http.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
    http.onreadystatechange = function() {//Call a function when the state changes.
        if(http.status != 200) {
            alert("No response");
        }
    }    
    http.send(params);
}

function alertContents(http_request)
{
	if (http_request.readyState == 4)
    {
		if (http_request.status == 200)
        {
            parse_vars(http_request.responseText);
        }
        data_received = 0;
    }
}

function check_voice_cmd(url)
{
    var http_request = false;

	data_received = 1;
    http_request = new XMLHttpRequest();
    http_request.onreadystatechange = function() { checkContents(http_request); };
    http_request.open('GET', url, true);
    http_request.send(null);
}

function checkContents(http_request)
{
    if (http_request.readyState == 4)
    {
		if (http_request.status == 200)
        {
            parse_voice_data(http_request.responseText);
        }
        data_received = 0;
    }
}

function switch2SensorPage() {
    window.location.href = "sensorPoll.html";
}


function parse_voice_data(data) {
    var object1 = JSON.parse(data);
    if (object1.voiceCmd == 1) {
        switch2SensorPage();
    }
}