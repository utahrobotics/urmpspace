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
