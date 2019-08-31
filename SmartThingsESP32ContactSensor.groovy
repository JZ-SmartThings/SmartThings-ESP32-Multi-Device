/**
 *  ESP32 Contact Sensor v1.0.20190831
 *  Copyright 2019 JZ
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
 *  in compliance with the License. You may obtain a copy of the License at:
 *      http://www.apache.org/licenses/LICENSE-2.0
 *  Unless required by applicable law or agreed to in writing, software distributed under the License is distributed
 *  on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License
 *  for the specific language governing permissions and limitations under the License.
 */
metadata {
	definition (name: "ESP32 Contact Sensor", namespace: "JZ", author: "JZ") {
		capability "Contact Sensor"
		capability "Sensor"
		capability "Switch"
		capability "Health Check"
		capability "Temperature Measurement"
		capability "Relative Humidity Measurement"

		command "open"
		command "close"
		command "on"
		command "off"
        command "markDeviceOnline"
        command "markDeviceOffline"
		command "setTemperature"
		command "setHumidity"
	}
	simulator {
		status "open": "contact:open"
		status "closed": "contact:closed"
	}
	tiles {
		standardTile("contact", "device.contact", width: 3, height: 2) {
			state("closed", label:'${name}', icon:"st.contact.contact.closed", backgroundColor:"#53a7c0")
			state("open", label:'${name}', icon:"st.contact.contact.open", backgroundColor:"#FF6600")
		}
		standardTile("switch", "device.switch", width: 1, height: 1, canChangeIcon: true) { // SWITCH NOT SHOWN BUT USED TO SYNC CONTACT STATE AS IT'S READ ONLY OTHERWISE
			state "off", label: '${currentValue}', action: "switch.on", icon: "st.switches.switch.off", backgroundColor: "#ffffff"
			state "on", label: '${currentValue}', action: "switch.off", icon: "st.switches.switch.on", backgroundColor: "#79b821"
		}
        standardTile("deviceHealthControl", "device.healthStatus", decoration: "flat", width: 1, height: 1, inactiveLabel: false) {
            state "online",  label: "ONLINE", backgroundColor: "#00A0DC", action: "markDeviceOffline", icon: "st.Health & Wellness.health9", nextState: "goingOffline", defaultState: true
            state "offline", label: "OFFLINE", backgroundColor: "#E86D13", action: "markDeviceOnline", icon: "st.Health & Wellness.health9", nextState: "goingOnline"
            state "goingOnline", label: "Going ONLINE", backgroundColor: "#FFFFFF", icon: "st.Health & Wellness.health9"
            state "goingOffline", label: "Going OFFLINE", backgroundColor: "#FFFFFF", icon: "st.Health & Wellness.health9"
        }
        valueTile("temperature", "device.temperature", width: 1, height: 1, decoration: "flat") {
            state "temperature", label:'${currentValue}°', unit:"F", icon:"st.Weather.weather2",
                backgroundColors:[
                // Celsius
            	[value: 0, color: "#153591"],
                [value: 7, color: "#1e9cbb"],
                [value: 15, color: "#90d2a7"],
                [value: 23, color: "#44b621"],
                [value: 28, color: "#f1d801"],
                [value: 35, color: "#d04e00"],
                [value: 37, color: "#bc2323"],
                // Fahrenheit
                [value: 40, color: "#153591"],
                [value: 44, color: "#1e9cbb"],
                [value: 63, color: "#90d2a7"],
                [value: 74, color: "#44b621"],
                [value: 80, color: "#f1d801"],
                [value: 95, color: "#d04e00"],
                [value: 96, color: "#bc2323"]
            ]
        }
		standardTile("humidity", "device.humidity", width:1, height:1, inactiveLabel: false, decoration: "flat") {
			state "humidity", label:'${currentValue}%', icon:"st.Weather.weather12"
		}
		main "contact"
		details (["contact","deviceHealthControl","temperature","humidity"])
	}
}

def installed() {
    log.trace "Executing 'installed'"
    markDeviceOnline()
    off()
    initialize()
}

def updated() {
    log.trace "Executing 'updated'"
    initialize()
}

def markDeviceOnline() {
    setDeviceHealth("online")
}

def markDeviceOffline() {
    setDeviceHealth("offline")
}

private setDeviceHealth(String healthState) {
    log.debug("healthStatus: ${device.currentValue('healthStatus')}; DeviceWatch-DeviceStatus: ${device.currentValue('DeviceWatch-DeviceStatus')}")
    // ensure healthState is valid
    List validHealthStates = ["online", "offline"]
    healthState = validHealthStates.contains(healthState) ? healthState : device.currentValue("healthStatus")
    // set the healthState
    sendEvent(name: "DeviceWatch-DeviceStatus", value: healthState)
    sendEvent(name: "healthStatus", value: healthState)
}

private initialize() {
    log.trace "Executing 'initialize'"
    sendEvent(name: "DeviceWatch-Enroll", value: [protocol: "cloud", scheme:"untracked"].encodeAsJson(), displayed: false)
	defaultTempHumidity()
}

def parseOLD(String description) {
	def pair = description.split(":")
	createEvent(name: pair[0].trim(), value: pair[1].trim())
}

def parse(description) {
	log.debug description
	def eventMap
	if (description.type == null) eventMap = [name:"$description.name", value:"$description.value"]
	else eventMap = [name:"$description.name", value:"$description.value", type:"$description.type"]
	createEvent(eventMap)
}

def open() {
	log.trace "open()"
	sendEvent(name: "contact", value: "open")
}

def close() {
	log.trace "close()"
    sendEvent(name: "contact", value: "closed")
}

def on() {
	log.debug "$version on()"
	sendEvent(name: "switch", value: "on")
	sendEvent(name: "contact", value: "open")
}

def off() {
	log.debug "$version off()"
	sendEvent(name: "switch", value: "off")
    sendEvent(name: "contact", value: "closed")
}

def setTemperature(temp) {
	sendEvent(name: "temperature", value: temp)
}

def setHumidity(hmdt) {
	sendEvent(name: "humidity", value: hmdt)
}

def defaultTempHumidity() {
	if (device.currentState("temperature")==null) {
    	log.debug "temperature is null, defaulting -99"
    	sendEvent(name: "temperature", value: -99)
    }
	if (device.currentState("humidity")==null) {
    	log.debug "humidity is null, defaulting -99"
    	sendEvent(name: "humidity", value: -99)
    }
}