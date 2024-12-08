import { socket } from '../shared/styles/scripts/socket';
import React, { useState } from 'react';
import { Row, Col, Form } from 'react-bootstrap';

const CameraRotationControl = () => {
  const [droneControl, setDroneControl] = useState({
    pitch: 27,
    yaw: 5,
    roll: 29,
    x: -1.25,
    y: 2.25,
    z: -1.2
  });

  // Function to emit the updated drone control values to the backend
  const emitControlUpdate = (updatedControl) => {
    socket.emit("update-camera-rotation", updatedControl);
  };

  // Handle change for each field (rotation and position) and emit the update
  const handleControlChange = (field, value) => {
    const updatedValue = parseFloat(value) || 0;
    const updatedControl = { ...droneControl, [field]: updatedValue };
    setDroneControl(updatedControl);
    emitControlUpdate(updatedControl); // Emit to backend
  };

  return (
    <div>
      <Row className='pt-3'>
        <Col xs={{ offset: 2 }} className='text-center'>
          Pitch
        </Col>
        <Col className='text-center'>
          Yaw
        </Col>
        <Col className='text-center'>
          Roll
        </Col>
      </Row>

      <Row className='pt-2'>
        <Col xs={2} className='pt-2 text-end'>
          Rotation
        </Col>
        <Col>
          <Form.Control
            type="number"
            step="1" // Step changed to 0.05
            value={droneControl.pitch}
            onChange={(event) => handleControlChange("pitch", event.target.value)}
          />
        </Col>
        <Col>
          <Form.Control
            type="number"
            step="1" // Step changed to 0.05
            value={droneControl.yaw}
            onChange={(event) => handleControlChange("yaw", event.target.value)}
          />
        </Col>
        <Col>
          <Form.Control
            type="number"
            step="1" // Step changed to 0.05
            value={droneControl.roll}
            onChange={(event) => handleControlChange("roll", event.target.value)}
          />
        </Col>
      </Row>

      <Row className='pt-3'>
        <Col xs={{ offset: 2 }} className='text-center'>
          X
        </Col>
        <Col className='text-center'>
          Y
        </Col>
        <Col className='text-center'>
          Z
        </Col>
      </Row>

      <Row className='pt-2'>
        <Col xs={2} className='pt-2 text-end'>
          Position
        </Col>
        <Col>
          <Form.Control
            type="number"
            step="0.05" // Step changed to 0.05
            value={droneControl.x}
            onChange={(event) => handleControlChange("x", event.target.value)}
          />
        </Col>
        <Col>
          <Form.Control
            type="number"
            step="0.05" // Step changed to 0.05
            value={droneControl.y}
            onChange={(event) => handleControlChange("y", event.target.value)}
          />
        </Col>
        <Col>
          <Form.Control
            type="number"
            step="0.05" // Step changed to 0.05
            value={droneControl.z}
            onChange={(event) => handleControlChange("z", event.target.value)}
          />
        </Col>
      </Row>
    </div>
  );
};

export default CameraRotationControl;
