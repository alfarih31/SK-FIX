var socket = io.connect(`http://${document.location.hostname}:2000`, {'Access-Control-Allow-Origin' : '*:*'});

socket.on('connect', function(msg) {
    var myJSON = checkCookie();
    document.getElementById('CT').innerHTML = "Not running"
    socket.emit('req', JSON.stringify(myJSON));
});

socket.on('update', (msg) => {
    const msg_time = JSON.parse(msg);

    var button = document.getElementById('L_S').disabled;

    if (button){
        document.getElementById('CT').innerHTML = msg_time['CT'] + "/" + msg_json['MT'];
    }
});

socket.on('res', (msg) => {
    const msg_json = JSON.parse(msg);
    const status = msg_json['status']
    var ssid = getCookie("ssid");
    if (ssid == " " || ssid == 0 || ssid == undefined){
        setCookie("ssid", msg_json['ssid'],365);
    }

    const submit = msg_json['SUBMIT'];

    const is_warn = msg_json['WARN'];
    if (is_warn == true) {
        const alert_msg = msg_json['msg'];
        alert(alert_msg);
    }

//    document.getElementById('L_S').disabled = !msg_json['L_S'];
//    document.getElementById('L_STP').disabled = !msg_json['L_STP'];
    document.getElementById('I_S').disabled = !msg_json['I_S'];
    document.getElementById('I_STP').disabled = !msg_json['I_STP'];
    document.getElementById('M_S').disabled = !msg_json['M_S'];
    document.getElementById('M_STP').disabled = !msg_json['M_STP'];
    document.getElementById('tombolPN').disabled = !submit;
    document.getElementById('projek').disabled = !submit;

    if (msg_json["M_STP"]) {
	document.getElementById('CT').innerHTML = "Running"
    }

    if (status['I']) {
        document.getElementById('P_I').innerHTML = "Found Sensor";
        document.getElementById('P_I').style.color = "green";
    }
    if (status['M']) {
        document.getElementById('P_M').innerHTML = "Found USB";
        document.getElementById('P_M').style.color = "green"
    };
    document.getElementById('PN').innerHTML = msg_json['name'];
});

function checkCookie() {
    var ssid = getCookie("ssid");
    if (ssid == " " || ssid == undefined){
        ssid = '0'
    }
    var myObj = {ssid};
    return myObj;
}

function setCookie(cname, cvalue, exdays) {
    var d = new Date();
    d.setTime(d.getTime() + (exdays*24*60*60*1000));
    var expires = "expires="+ d.toUTCString();
    document.cookie = cname + "=" + cvalue + ";" + expires + ";path=/";
}

function getCookie(cname) {
    var name = cname + "=";
    var decodedCookie = decodeURIComponent(document.cookie);
    var ca = decodedCookie.split(';');
    for(var i = 0; i <ca.length; i++) {
        var c = ca[i];
        while (c.charAt(0) == ' ') {
            c = c.substring(1);
        }
        if (c.indexOf(name) == 0) {
        return c.substring(name.length, c.length);
        }
    }
    return 0;
}

function tombolL_S(){
//    socket.emit('button', "L_S");
}

function tombolL_STP(){
//    socket.emit('button', "L_STP");
}

function tombolI_S(){
    socket.emit('button', "I_S");
}

function tombolI_STP(){
    socket.emit('button', "I_STP");
}

function tombolM_S(){
    socket.emit('button', "M_S");
}

function tombolM_STP(){
    socket.emit('button', "M_STP");
}

function tombolSBMT(){
    var isi = document.getElementById("isitext").value;
    document.getElementById("tombolPN").click();
    socket.emit('cname', isi);
}



