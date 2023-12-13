// Decode uplink function.
//
// Input is an object with the following fields:
// - bytes = Byte array containing the uplink payload, e.g. [255, 230, 255, 0]
// - fPort = Uplink fPort.
// - variables = Object containing the configured device variables.
//
// Output must be an object with the following fields:
// - data = Object representing the decoded payload.
function decodeUplink(input) {
  const data = decodeDataBytes(input);
  
 
    return {
      data,
    };
}
function decodeDataBytes(input) {
  // Convert Latitude bytes to Int and then to deg
  const latitudeBytes =
    (input.bytes[0] << 16) | (input.bytes[1] << 8) | input.bytes[2];
  const latitude = (latitudeBytes / 16777215) * 180 - 90;
  // Convert Longitude bytes to Int and then to deg
  const longitudeBytes =
    (input.bytes[3] << 16) | (input.bytes[4] << 8) | input.bytes[5];
  const longitude = (longitudeBytes / 16777215) * 360 - 180;
  // Convert Altitude Bytes to M
  //Convert HdopGPS
  const hdop = input.bytes[6] / 10;
  //Convert Stats
  const stats = input.bytes[7];
  //Convert Time
  const timeBytes =
    (input.bytes[8] << 16) | (input.bytes[9] << 8) | input.bytes[10];
  const timeHrs = Math.floor(timeBytes / 3600);
  const timeMin = Math.floor((timeBytes % 3600) / 60);
  const timeSec = Math.floor(timeBytes % 60);
  //Convert Battery pourcentage
  const batteryPercentage = input.bytes[11];;
  //battery status
  const batteryStatusBytes = input.bytes[12];
  const batStatus = batteryStatusBytes == 1 ? "Charging" : "Not Charging";

  return {
    latitude,
    longitude,
    hdop,
    stats,
    timeHrs,
    timeMin,
    timeSec,
    batteryPercentage,
    batStatus,
  };
}