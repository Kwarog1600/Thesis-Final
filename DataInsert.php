<?php
$timestamp = date("Y-m-d H:i:s");
$temperature = $_POST['temperature'];
$pulseRate = $_POST['pulseRate'];
$SPO2 = $_POST['spo2'];
$wearStatus = $_POST['wearStatus'];
$Battery = $_POST['Battery'];
$DevID = $_POST['DevID'];
$wearStatusInt = $wearStatus ? 1 : 0;
$notes = "";

// Database credentials
$servername = "localhost";
$username = "root";
$password = "";
$dbname = "monitoring system";

// Create connection
$conn = mysqli_connect($servername, $username, $password, $dbname);

// Check connection
if (!$conn) {
    die("Connection failed: " . mysqli_connect_error());
}

// Check wearStatus from devicelist table
$wearStatus = false; // Default value

// Prepare the query to get the SurName and id of the patient info
$sql = "SELECT lastname, patientID FROM patient_info WHERE Dev_ID = '$DevID'";

// Execute the query and get the result
$result = mysqli_query($conn, $sql);

// Check if there is any result for the DevID of the device
if (mysqli_num_rows($result) > 0) {
    // Loop through the result and store the SurName and id in separate variables
    while($row = mysqli_fetch_assoc($result)) {
        $SurName = strtolower($row["lastname"]);
        $id = $row["patientID"];
    }
} else {
    echo "No results found";
}

// Pulse
if ($pulseRate < 59 || $pulseRate > 101) {
    $notes .= "Pulse rate is beyong or below normal values. ";
}

// Hall Sensor
if ($wearStatusInt === 0) {
    $notes .= "Hall sensor does not detect the strap. Possible attempt to remove.";
}

// SPO2
if ($SPO2 < 94 || $SPO2 > 101) {
    $notes .= "SPO2 is out of range. ";
}

// Temperature
if ($temperature < 36 || $temperature > 37.3) {
    $notes .= "Temperature is beyond or below normal values. ";
}

// RFID
// Assuming you have the RFID value, and if it's false, add a note
$RFID = false; // Replace with the actual RFID value
if (!$RFID) {
    $notes .= "Device is detected by the RFID Sensor. Possible Security breach.";
}

// Example: Check if the battery is low and add a note if necessary
$Battery = $_POST['Battery'];
if ($Battery < 20) {
    $notes .= "Battery is low. ";
}

// Insert data into the corresponding patient data table
$sql = "INSERT INTO {$SurName}{$id} (temperature, pulse, SPO2, wearStat, RFID, time_stamp, notes) VALUES ('$temperature', '$pulseRate', '$SPO2', '$wearStatusInt', '0',  '$timestamp', $notes)";

if ($conn->query($sql) === TRUE) {
    echo "Data added to database successfully";
} else {
    echo "Error: " . $sql . "<br>" . $conn->error;
}

// Update battery in the devicelist table
$updateBatterySQL = "UPDATE devicelist SET Battery = '$Battery' WHERE MAC = '$DevID'";
if ($conn->query($updateBatterySQL) === TRUE) {
    echo "Battery information updated successfully";
} else {
    echo "Error updating battery information: " . $conn->error;
}

// Close connection
mysqli_close($conn);
?>
