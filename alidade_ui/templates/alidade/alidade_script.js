// Widget Vizanti « Alidade » : etat du robot (/robot/state) et commande des
// modes (/mode_manager/set_mode, save_map, list_maps). Un seul panneau, pas
// de reglage a persister : tout vient du gestionnaire de modes.
let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let rosbridge = rosbridgeModule.rosbridge;
let Status = StatusModule.Status;

const MODE_NAMES = ["IDLE", "TELEOP", "MAPPING", "LOCALIZATION", "EXPLORATION", "DOCKING"];
const MODE_COLORS = ["#6b7a88", "#2f7d4f", "#0b7285", "#3b5bdb", "#a8650f", "#7048e8"];
const HEALTH = ["ok", "warn", "error", "stale"];

const icondiv = document.getElementById("{uniqueID}_icon");
const icon = icondiv.getElementsByTagName("img")[0];
const icontext = document.getElementById("{uniqueID}_icontext");
const status = new Status(icondiv, document.getElementById("{uniqueID}_status"));

const el = (name) => document.getElementById("{uniqueID}_" + name);
const modeSpan = el("mode"), substateSpan = el("substate"), mapSelect = el("map");
const saveName = el("savename"), saveButton = el("save");

function svgIcon(color, letter) {
	const svg = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 50 50">` +
		`<circle cx="25" cy="25" r="22" fill="${color}" stroke="#ffffff" stroke-width="2"/>` +
		`<text x="25" y="31" text-anchor="middle" font-family="sans-serif" font-size="18" font-weight="700" fill="#ffffff">${letter}</text></svg>`;
	return "data:image/svg+xml;charset=utf-8," + encodeURIComponent(svg);
}
icon.src = svgIcon(MODE_COLORS[0], "A");

let lastState = null;

function render(msg) {
	lastState = msg;
	const name = msg.mode_name || MODE_NAMES[msg.mode] || "?";
	icon.src = svgIcon(MODE_COLORS[msg.mode] || MODE_COLORS[0], name[0]);
	icontext.textContent = name;
	modeSpan.textContent = name + (msg.transitioning ? " (transition)" : "");
	substateSpan.textContent = msg.substate ? " - " + msg.substate : "";

	const q = Math.round((msg.localization_quality || 0) * 100);
	el("loc").textContent = msg.localization_source
		? `${msg.localization_source}, ${msg.localized ? "localise" : "pas de repere map"} (${q} %)`
		: "aucune";
	el("mapname").textContent = msg.map_name || "-";
	el("nav").textContent = msg.nav_active ? "actif" : "en pause";
	el("goal").textContent = msg.goal_active
		? `${msg.goal_status}, ${msg.distance_remaining.toFixed(2)} m restants`
		: (msg.goal_status || "aucun");
	const dot = (ok) => ok ? "OK" : "KO";
	el("sensors").textContent = `LiDAR ${dot(msg.lidar_ok)} · camera ${dot(msg.camera_ok)} · IMU ${dot(msg.imu_ok)} · Pico ${dot(msg.pico_ok)}`;
	el("health").textContent = HEALTH[msg.health] || "?";

	const errors = el("errors");
	errors.innerHTML = "";
	for (const e of msg.errors || []) {
		const li = document.createElement("li");
		li.textContent = e;
		errors.appendChild(li);
	}

	for (const b of document.querySelectorAll("#{uniqueID}_modes .alidade_mode")) {
		const active = parseInt(b.dataset.mode) === msg.mode;
		b.style.fontWeight = active ? "700" : "400";
		b.style.outline = active ? "2px solid " + MODE_COLORS[msg.mode] : "none";
		b.disabled = msg.transitioning;
	}

	if (msg.health === 2) status.setError("sante : erreur");
	else if (msg.health === 1) status.setWarn("sante : avertissement");
	else if (msg.transitioning) status.setWarn(msg.substate || "transition");
	else status.setOK();
}

const stateTopic = new ROSLIB.Topic({
	ros: rosbridge.ros,
	name: "/robot/state",
	messageType: "alidade_msgs/msg/RobotState",
	throttle_rate: 250,
	compression: rosbridge.compression,
	queue_length: 1
});
stateTopic.subscribe(render);

function callService(name, type, request, done) {
	const service = new ROSLIB.Service({ ros: rosbridge.ros, name: name, serviceType: type });
	service.callService(new ROSLIB.ServiceRequest(request), done, (err) => {
		status.setError(`${name} : ${err}`);
	});
}

async function refreshMaps() {
	callService("/mode_manager/list_maps", "alidade_msgs/srv/ListMaps", {}, (res) => {
		const current = mapSelect.value || res.current;
		mapSelect.innerHTML = "";
		for (const n of res.names) {
			const opt = document.createElement("option");
			opt.value = n; opt.textContent = n;
			mapSelect.appendChild(opt);
		}
		if (current && res.names.includes(current)) mapSelect.value = current;
	});
}

for (const b of document.querySelectorAll("#{uniqueID}_modes .alidade_mode")) {
	b.addEventListener("click", () => {
		const mode = parseInt(b.dataset.mode);
		const req = { mode: mode, map_name: mode === 3 ? (mapSelect.value || "") : "" };
		status.setWarn(`demande ${MODE_NAMES[mode]}`);
		// Le service repond des que la demande est acceptee ; la transition
		// elle-meme se suit dans /robot/state (mode, sous-etat).
		callService("/mode_manager/set_mode", "alidade_msgs/srv/SetMode", req, (res) => {
			if (res.ok) status.setOK(res.message); else status.setError(res.message);
		});
	});
}

saveButton.addEventListener("click", () => {
	status.setWarn("sauvegarde demandee");
	callService("/mode_manager/save_map", "alidade_msgs/srv/SaveMap", { name: saveName.value || "" }, (res) => {
		if (res.ok) { status.setOK(res.message); setTimeout(refreshMaps, 6000); } else status.setError(res.message);
	});
});

icon.addEventListener("click", refreshMaps);
refreshMaps();
setInterval(refreshMaps, 15000);
