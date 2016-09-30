$(document).ready(function() {
	checkWidth();
  if (debug === true) {
    toggleDebug();
  }
  writeConfigView();
});

$(window).resize(function(){
	checkWidth();
  if (debug === true) {
    toggleDebug();
  }
});

var mobile = true;

function checkWidth() {
	var windowsize = $(window).width();

	if (windowsize > 639) {
		mobile = false;
		return mobile;
	} else {
		mobile = true;
		return mobile;
	}
}

$("#debug").click(toggleDebug);

var debug = false;
var configView = $("#configView").val();

function toggleDebug() {
	if (mobile === true) {
		alert("Temporarily unavailable on mobile. ");
	} else if (debug === false) {
		debug = true;
		$("#debug").html(
			"<div class='debug'><i class='material-icons'>visibility_off</i></div>"
		);
		$("#debug-console").show();
        writeDebug();
	} else if (debug === true) {
		debug = false;
		$("#debug").html(
			"<div class='debug'><i class='material-icons'>visibility</i></div>"
		);
		$("#debug-console").hide();
	}
}

function writeDebug() {
  $("#configView").val(configView);
}

$("#configView").on("input", function() {
  updateConfigView();
});

function updateConfigView() {
  configView = $("#configView").val();
  localStorage.setItem('configViewStorage', configView);
  writeConfigView();
}

function writeConfigView() {
    if (localStorage.getItem('configViewStorage') !== null) {
      configView = localStorage.getItem('configViewStorage');
    }
    if (configView == "Complex") {
      topView =
        "<button type='button' class='flex-top'><i class='material-icons'>play_arrow</i></button>" +
        "<button type='button' class='flex-top'><i class='material-icons'>keyboard_arrow_up</i></button>" +
        "<button type='button' class='flex-top'><i class='material-icons'>fast_forward</i></button>";
      botView =
        "<button type='button' class='flex-bot'><i class='material-icons'>explore</i></button>" +
        "<button type='button' class='flex-bot'><i class='material-icons'>keyboard_arrow_down</i></button>" +
        "<button type='button' class='flex-bot'><i class='material-icons'>refresh</i></button>";
    } else if (configView == "Simple") {
      topView =
        "<button type='button' class='flex-top'><i class='material-icons'>keyboard_arrow_up</i></button>";
      botView =
        "<button type='button' class='flex-bot'><i class='material-icons'>keyboard_arrow_down</i></button>";
    }
    document.getElementById("top").innerHTML = topView;
    document.getElementById("bot").innerHTML = botView;
}

$("#mid").html(
	"<button type='button' class='flex-mid'><i class='material-icons'>undo</i></button>" +
	"<button type='button' class='flex-mid'><i class='material-icons'>stop</i></button>" +
	"<button type='button' class='flex-mid'><i class='material-icons'>redo</i></button>"
);

var d = new Date();

$("#footer").html(
	"<div class='footer'>" +
	"<div class='footer-label'><p>Contact</p></div>" +
	"<a href='mailto:zackrisongeorge@gmail.com'><img id='special' class='footer-icon' src='../icons/email.png' alt='Gmail'></a>" +
	"<a href='https://plus.google.com/u/0/+GeorgeZackrison'target='_blank'><img id='special' class='footer-icon' src='../icons/plus.png' alt='G+'></a>" +
	"<a href='https://twitter.com/ZackrisonGeorge' target='_blank'><img id='special' class='footer-icon' src='../icons/twitter.png' alt='Twitter'></a>" +
	"<div class='copyright'><p class='copyright-icon'><i class='material-icons'>copyright</i> " + (d.getFullYear()) + " George Zackrison</p><p class='copyright-label'>All rights reserved.</p></div>" +
	"</div>"
);
