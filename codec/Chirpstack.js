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
  const dataMissing = decodeMissingBytes(input);
  if (!checkMissing(dataMissing) && !checkButtons(input)) {
    return {
      data,
    };
  } else if (!checkButtons(input)) {
    const missingData = decodeMissingData(dataMissing);
    return {
      data: {
        latitude: data.altitude,
        longitude: data.longitude,
        altitude: data.altitude,
        hdop: data.hdop,
        stats: data.stats,
        timeHrs: data.timeHrs,
        timeMin: data.timeMin,
        timeSec: data.timeSec,
        batteryPercentage: data.batteryPercentage,
        batStatus: data.batStatus,
        missingData,
      },
    };
  } else if (!checkMissing(dataMissing)) {
    const buttonData = buttonsData(input);
    return {
      data: {
        latitude: data.altitude,
        longitude: data.longitude,
        altitude: data.altitude,
        hdop: data.hdop,
        stats: data.stats,
        timeHrs: data.timeHrs,
        timeMin: data.timeMin,
        timeSec: data.timeSec,
        batteryPercentage: data.batteryPercentage,
        batStatus: data.batStatus,
        buttonData,
      },
    };
  } else {
    const missingData = decodeMissingData(dataMissing);
    const buttonData = buttonsData(input);
    return {
      data: {
        latitude: data.altitude,
        longitude: data.longitude,
        altitude: data.altitude,
        hdop: data.hdop,
        stats: data.stats,
        timeHrs: data.timeHrs,
        timeMin: data.timeMin,
        timeSec: data.timeSec,
        batteryPercentage: data.batteryPercentage,
        batStatus: data.batStatus,
        missingData,
        buttonData,
      },
    };
  }
}

// Encode downlink function.
//
// Input is an object with the following fields:
// - data = Object representing the payload that must be encoded.
// - variables = Object containing the configured device variables.
//
// Output must be an object with the following fields:
// - bytes = Byte array containing the downlink payload.
function encodeDownlink(input) {}

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
  const altitude = (input.bytes[6] << 8) | input.bytes[7];
  //Convert HdopGPS
  const hdop = input.bytes[8] / 10;
  //Convert Stats
  const stats = input.bytes[9];
  //Convert Time
  const timeBytes =
    (input.bytes[10] << 16) | (input.bytes[11] << 8) | input.bytes[12];
  const timeHrs = Math.floor(timeBytes / 3600);
  const timeMin = Math.floor((timeBytes % 3600) / 60);
  const timeSec = Math.floor(timeBytes % 60);
  //Convert Battery pourcentage
  const batteryBytes = input.bytes[13];
  const batteryPercentage = Math.floor((batteryBytes / 255) * 100) + "%";
  //battery status
  const batteryStatusBytes = input.bytes[14];
  const batStatus = batteryStatusBytes == 1 ? "Charging" : "Not Charging";

  return {
    latitude,
    longitude,
    altitude,
    hdop,
    stats,
    timeHrs,
    timeMin,
    timeSec,
    batteryPercentage,
    batStatus,
  };
}

function decodeMissingBytes(input) {
  const latMissingBytes =
    (input.bytes[15] << 16) | (input.bytes[16] << 8) | input.bytes[17];
  const longMissingBytes =
    (input.bytes[18] << 16) | (input.bytes[19] << 8) | input.bytes[20];
  const altMissing = (input.bytes[21] << 8) | input.bytes[22];
  const hdopMissingBytes = input.bytes[23];
  const statMissingBytes = input.bytes[24];
  const timeMissingBytes =
    (input.bytes[25] << 16) | (input.bytes[26] << 8) | input.bytes[27];
  const batMissingBytes = input.bytes[28];
  const batteryStatusMissingBytes = input.bytes[29];
  return {
    latMissingBytes,
    longMissingBytes,
    altMissing,
    hdopMissingBytes,
    statMissingBytes,
    timeMissingBytes,
    batMissingBytes,
    batteryStatusMissingBytes,
  };
}
function decodeMissingData(dataMissing) {
  const latMissing = (dataMissing.latMissingBytes / 16777215) * 180 - 90;
  const longMissing = (dataMissing.longMissingBytes / 16777215) * 360 - 180;
  const hdpMissing = dataMissing.hdopMissingBytes / 10;
  const timeHrsMissing = Math.floor(dataMissing.timeMissingBytes / 3600);
  const timeMinMissing = Math.floor((dataMissing.timeMissingBytes % 3600) / 60);
  const timeSecMissing = Math.floor(dataMissing.timeMissingBytes % 60);
  const batPercentageMissing =
    Math.floor((dataMissing.batMissingBytes / 255) * 100) + "%";
  const batStatusMissing =
    dataMissing.batteryStatusMissingBytes == 1 ? "Charging" : "Not Charging";
  return {
    latMissing,
    longMissing,
    altMissing: dataMissing.altMissing,
    hdpMissing,
    statMissingBytes,
    timeHrsMissing,
    timeMinMissing,
    timeSecMissing,
    batPercentageMissing,
    batStatusMissing,
  };
}

function checkMissing(dataMissing) {
  return !(
    dataMissing.latMissingBytes == 0 &&
    dataMissing.longMissingBytes == 0 &&
    dataMissing.altMissing == 0 &&
    dataMissing.hdopMissingBytes == 0 &&
    dataMissing.statMissingBytes == 0 &&
    dataMissing.timeMissingBytes == 0 &&
    dataMissing.batMissingBytes == 0 &&
    dataMissing.batteryStatusMissingBytes == 0
  );
}
function readButtonBytes(input, position) {
  const buttonNum = input.bytes[position + 1];
  const timeButton =
    (input.bytes[position + 2] << 16) |
    (input.bytes[position + 3] << 8) |
    input.bytes[position + 4];
  const latButton =
    (input.bytes[position + 5] << 16) |
    (input.bytes[position + 6] << 8) |
    input.bytes[position + 7];
  const longButton =
    (input.bytes[position + 8] << 16) |
    (input.bytes[position + 9] << 8) |
    input.bytes[position + 10];

  return {
    buttonNum,
    timeButton,
    latButton,
    longButton,
  };
}

function decodeButtonsData(dataButtons) {
  const latitudeButton = (dataButtons.latButton / 16777215) * 180 - 90;
  const lonButton = (dataButtons.longButton / 16777215) * 360 - 180;
  const timeHrsButton = Math.floor(dataButtons.timeButton / 3600);
  const timeMinButton = Math.floor((dataButtons.timeButton % 3600) / 60);
  const timeSecButton = Math.floor(dataButtons.timeButton % 60);
  

  return {
    buttonNum: dataButtons.buttonNum,
    timeHrsButton,
    timeMinButton, 
    timeSecButton,
    latitudeButton,
    lonButton,
  };
}

function buttonsData(input) {
  const numOfButton = input.bytes[30];
  switch (numOfButton) {
    case 1: {
      const button1Bytes = readButtonBytes(input, 30);
      const button1 = decodeButtonsData(button1Bytes); 
      return { button1Bytes, button1 };
    }

    case 2: {
      const button1Bytes = readButtonBytes(input, 30);
      const button2Bytes = readButtonBytes(input, 40);
      const button1 = decodeButtonsData(button1Bytes);
      const button2 = decodeButtonsData(button2Bytes);
      return { button1Bytes , button2Bytes , button1, button2 };
    }

    case 3: {
      const button1Bytes = readButtonBytes(input, 30);
      const button2Bytes = readButtonBytes(input, 40);
      const button3Bytes = readButtonBytes(input, 50);
      const button1 = decodeButtonsData(button1Bytes);
      const button2 = decodeButtonsData(button2Bytes);
      const button3 = decodeButtonsData(button3Bytes);
      return { button1, button2, button3 };
    }

    case 4: {
      const button1Bytes = readButtonBytes(input, 30);
      const button2Bytes = readButtonBytes(input, 40);
      const button3Bytes = readButtonBytes(input, 50);
      const button4Bytes = readButtonBytes(input, 60);
      const button1 = decodeButtonsData(button1Bytes);
      const button2 = decodeButtonsData(button2Bytes);
      const button3 = decodeButtonsData(button3Bytes);
      const button4 = decodeButtonsData(button4Bytes);
      return { button1, button2, button3, button4 };
    }

    default:
      break;
  }
}



function checkButtons(input) {
  return !(
    input.bytes[30] == 0
  );
}
