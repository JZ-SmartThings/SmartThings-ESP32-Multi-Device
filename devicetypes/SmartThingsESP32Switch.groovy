/**
 *  ESP32 Switch v1.0.20190831
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

    definition (name: "ESP32 Switch", namespace: "JZ", author: "JZ") {
        capability "Switch"
        capability "Relay Switch"
        capability "Sensor"
        capability "Actuator"
        capability "Health Check"
		capability "Temperature Measurement"
		capability "Relative Humidity Measurement"

        command "onPhysical"
        command "offPhysical"
        command "markDeviceOnline"
        command "markDeviceOffline"
		command "setTemperature"
		command "setHumidity"
    }

    tiles {
        standardTile("switch", "device.switch", width: 2, height: 2, canChangeIcon: true) {
            state "off", label: '${currentValue}', action: "switch.on", icon: "st.switches.switch.off", backgroundColor: "#ffffff"
            state "on", label: '${currentValue}', action: "switch.off", icon: "st.switches.switch.on", backgroundColor: "#00A0DC"
        }
        standardTile("on", "device.switch", decoration: "flat") {
            state "default", label: 'On', action: "onPhysical", backgroundColor: "#ffffff"
        }
        standardTile("off", "device.switch", decoration: "flat") {
            state "default", label: 'Off', action: "offPhysical", backgroundColor: "#ffffff"
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
        main "switch"
        details(["switch","on","off","deviceHealthControl","temperature","humidity"])
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

def on() {
    log.debug "$version on()"
    sendEvent(name: "switch", value: "on")
}

def off() {
    log.debug "$version off()"
    sendEvent(name: "switch", value: "off")
}

def onPhysical() {
    log.debug "$version onPhysical()"
    sendEvent(name: "switch", value: "on", type: "physical")
}

def offPhysical() {
    log.debug "$version offPhysical()"
    sendEvent(name: "switch", value: "off", type: "physical")
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

private getVersion() {
    "PUBLISHED"
}