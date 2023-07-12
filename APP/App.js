import React from "react";
import { useState, useEffect, useRef } from "react";
import { Text, View, TextInput, TouchableOpacity, Image } from "react-native";
import { StyleSheet } from "react-native";

import { initializeApp } from "firebase/app";
import { getAnalytics } from "firebase/analytics";
import firebase from 'firebase/app';
import { getDatabase, ref, onValue } from "firebase/database";
import emailjs from '@emailjs/browser';

import MaterialCommunityIcons from 'react-native-vector-icons/MaterialCommunityIcons';
const colors = {
  accent: '#FA754C',
  black: '#292934',
  white: '#FFFFFF',
  gray: '#A7A7A7',
  gray2: '#ECEDEF',
  button: '#F4F5F7',
};
var state_email = true;
var cond = [true, true, true]; 

export default function App() {

  const [temp, setTemp] = useState();
  const [pulse_rate, setPulseRate] = useState();
  const [spO2, setSpO2] = useState();
  const [stateT, setStateT] = useState();
  const [stateP, setStateP] = useState();
  const [stateS, setStateS] = useState();
  const [stateG, setStateG] = useState();
  const [general, setGeneral] = useState();

  useEffect(() => {
    const firebaseConfig = {
      apiKey: "AIzaSyDfKhznqHrYWK7lk1VLxxUY87i8eRYGkHU",
      authDomain: "test-c8474.firebaseapp.com",
      databaseURL: "https://test-c8474-default-rtdb.firebaseio.com",
      projectId: "test-c8474",
      storageBucket: "test-c8474.appspot.com",
      messagingSenderId: "748006346635",
      appId: "1:748006346635:web:dcb3a5881b6719cad9a98e",
      measurementId: "G-R1X9L45SGW"
    };

    // Initialize Firebase

    const app = initializeApp(firebaseConfig);
    const db = getDatabase(app);

    function read() {
      const starCountRef = ref(db);
      onValue(starCountRef, (snapshot) => {
        const data = snapshot.val();
        setTemp(data.temperature);
        setPulseRate(data.pulse_rate);
        setSpO2(data.spO2);

        var templateParams = {
          N: data.latitude,
          E: data.longtitude
        };
        function sendEmail() {
            emailjs.send('service_q4g13ou', 'template_7tcpy6l', templateParams, 'n4xgC_kLlVtmohWYr')
            .then(function(response) {
              console.log('SUCCESS!', response.status, response.text);
           }, function(error) {
              console.log('FAILED...', error);
           });
        }

        if (((data.temperature < 36) || (data.temperature > 37.5)) && (data.temperature != 0)) {
          changeColor('temp', 'red');
          cond[0] = false;
        } else {
          changeColor('temp', 'green');
          cond[0] = true;
        }

        if (((data.pulse_rate < 60) || (data.pulse_rate > 100)) && (data.pulse_rate != 0)) {
          changeColor('pulse', 'red');
          cond[1] = false;
        } else {
          changeColor('pulse', 'green');
          cond[1] = true;

        }

        if (((data.spO2 < 95) || (data.spO2 > 100)) && (data.spO2 != 0)) {
          changeColor('spO2', 'red');
          cond[2] = false;
        } else {
          changeColor('spO2', 'green');
          cond[2] = true;
        }
        if ((cond[0] == true) && (cond[1] == true) && (cond[2] == true)) {
          setGeneral('Tốt');
          setStateG('green');
        } else {
          setGeneral('Cảnh báo');
          setStateG('red');
        }
        function changeColor(state, c) {
          if (c == 'red' && state_email) {
            sendEmail();
            state_email = false;
          }
          if (state == 'temp')
            setStateT(c);
          if (state == 'pulse')
            setStateP(c);
          if (state == 'spO2')
            setStateS(c);
        }
      });
    }
    setInterval(() => {
      read();
    }, 1000);
    setInterval(() => {
      state_email = true;
    }, 5000);
    
  }, [])

  return (
    <>
      <View style={{ backgroundColor: '#ABB2B9' }}>
        <View >
          <Text style={styles.h3}>Giám sát sức khỏe</Text>

        </View>
        <View style={{ paddingVertical: 50 }}>
          <View style={{ paddingLeft: 1 }}>
            <Text style={styles.h2}>Tổng Quan Sức Khỏe</Text>
          </View>
          <View>
            <Text style={{
              fontSize: 45,
              color: stateG,
              textAlign: 'center'
            }}>{general}</Text>
          </View>
        </View>

        <View style={{ flexDirection: 'column' }}>
          <View style={styles.main}>
            <View style={styles.info}>
              <MaterialCommunityIcons name="heart-pulse" color="#2E86C1" size={50} />
              <Text style={styles.h4}>Nhịp tim</Text>
              <Text style={{
                color: stateP,
                fontSize: 20
              }}>{pulse_rate} bpm</Text>
              <Text>&#60;60-100&#62;</Text>
            </View>
            <View style={styles.info}>
              <MaterialCommunityIcons name="pulse" color="#2E86C1" size={50} />
              <Text style={styles.h4}>SPO2</Text>
              <Text style={{
                color: stateS,
                fontSize: 20
              }}>{spO2} %</Text>
              <Text>&#60;95-100&#62;</Text>
            </View>

          </View>
          <View style={styles.main}>
            <View style={styles.info}>
              <MaterialCommunityIcons name="temperature-celsius" color="#2E86C1" size={50} />
              <Text style={styles.h4}>Nhiệt độ</Text>
              <Text style={{
                color: stateT,
                fontSize: 20
              }}>{temp} &#186;C</Text>
              <Text>&#60;36-37.5&#62;</Text>
            </View>
          </View>

        </View>

        <View>
          <View style={styles.bottom} />

          <Text style={styles.h3}>TU_MO</Text>

        </View>

      </View>
    </>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#fff',
    alignItems: "center",
    justifyContent: "center",
  },
  tes: {
    backgroundColor: '#55B4C2',
    padding: 15
  },
  main: {
    flexDirection: 'row',
    paddingVertical: 25,
    justifyContent: 'space-around'

  },
  info: {
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: '#A7A7A7',
    width: 151,
    height: 151,
    borderRadius: 151 / 2,
    borderWidth: 1,
    borderColor: 'gray'
  },
  h2: {
    fontSize: 25,
    color: 'black',
    textAlign: 'center'
  },
  h3: {
    fontSize: 15,
    textAlign: 'center'
  },
  h4: {
    fontSize: 15,
    color: '#292934',
    textAlign: 'center'
  },
  bottom: {
    borderBottomColor: '#292934',
    marginTop: 75,
    borderBottomWidth: 1
  },
  image: {
    width: 40,
    height: 40,
    justifyContent: 'space-between',
    resizeMode: 'contain'
  }
})