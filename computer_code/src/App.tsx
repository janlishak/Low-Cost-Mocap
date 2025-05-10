"use client";

import { FormEventHandler, useState, createRef, Ref, useRef, useEffect } from 'react';
import { Button, Card, Col, Container, Row } from 'react-bootstrap';
import Toolbar from './components/Toolbar';
import Form from 'react-bootstrap/Form';
import { Tooltip } from 'react-tooltip'
import CameraWireframe from './components/CameraWireframe';
import FilledTriangle from './components/MySimplePlane';
import { io } from 'socket.io-client';
import { Canvas, useFrame } from '@react-three/fiber'
import { Stats, OrbitControls, Plane } from '@react-three/drei'
import Points from './components/Points';
import { socket } from './shared/styles/scripts/socket';
import { matrix, mean, multiply, rotationMatrix } from 'mathjs';
import Objects from './components/Objects';
import Chart from './components/chart';
import TrajectoryPlanningSetpoints from './components/TrajectoryPlanningSetpoints';
import CameraRotationControl from './components/CameraRotation';

const TRAJECTORY_PLANNING_TIMESTEP = 0.05
const LAND_Z_HEIGHT = 0.075
const NUM_DRONES = 2

export default function App() {
  const [cameraStreamRunning, setCameraStreamRunning] = useState(false);

  const [exposure, setExposure] = useState(100);
  const [gain, setGain] = useState(0);
  const [saveNumber, setSaveNumber] = useState(0);

  const [capturingPointsForPose, setCapturingPointsForPose] = useState(false);
  const [capturedPointsForPose, setCapturedPointsForPose] = useState("");

  const [isTriangulatingPoints, setIsTriangulatingPoints] = useState(false);
  const [isLocatingObjects, setIsLocatingObjects] = useState(false);

  const objectPoints = useRef<Array<Array<Array<number>>>>([])

  const trianglePointsRef = useRef<number[][]>([
    [0, 0.1, 0],    // Default vertex 1
    [-0.1, -0.1, 0], // Default vertex 2
    [0.1, -0.1, 0]   // Default vertex 3
  ]);
  
  const filteredObjects = useRef<object[][]>([])
  const droneSetpointHistory = useRef<number[][]>([])
  const objectPointErrors = useRef<Array<Array<number>>>([])
  const objects = useRef<Array<Array<Object>>>([])
  const [objectPointCount, setObjectPointCount] = useState(0);

  const [fps, setFps] = useState(0);

  const defaultPoses = [{"R":[[1,0,0],[0,1,0],[0,0,1]],"t":[0,0,0]},{"R":[[-0.525474380056056,-0.547647605294846,0.6511211686925283],[0.28508576450761886,0.6077203263898932,0.7412166429375882],[-0.8016250886666471,0.575115732098664,-0.16321492552094521]],"t":[-1.4144702501747746,-1.7228553395009831,2.3014089955372707]},{"R":[[-0.9799781568769824,0.16597551967980356,-0.10997699264396797],[0.015353091119408607,0.6137049991260283,0.7893861264557419],[0.19851220274096848,0.7718926744792572,-0.6039657311869505]],"t":[-0.0487301022379123,-1.868650748905911,3.5335333046779036]},{"R":[[0.43642217836309904,0.18824603644552773,-0.8798290242969689],[-0.32173544346159066,0.9458624928757966,0.04278374681297671],[0.8402511449878427,0.2644004053199093,0.4733607915884951]],"t":[1.9604400786356686,-0.4488835860622306,1.400191757450308]}];
  // const defaultWorldMatrix = [[0.8876159753616067,0.14303328857745706,0.43781201290232036,-1.25],[0.07765633540864646,0.8904744035902379,-0.44835792635071137,2.25],[-0.45399049973954675,0.4319685346287714,0.7792918652449212,-1.2],[0,0,0,1]];
  const defaultWorldMatrix =[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];

  const [cameraPoses, setCameraPoses] = useState<Array<object>>(defaultPoses)
  const [toWorldCoordsMatrix, setToWorldCoordsMatrix] = useState<number[][]>(defaultWorldMatrix)

  const [currentDroneIndex, setCurrentDroneIndex] = useState(0)
  const [droneArmed, setDroneArmed] = useState(Array.apply(null, Array(NUM_DRONES)).map(() => (false)))
  const [dronePID, setDronePID] = useState(["1", "0", "0", "1.5", "0", "0", "0.3", "0.1", "0.05", "0.2", "0.03", "0.05", "0.3", "0.1", "0.05", "28", "-0.035"])
  const [droneSetpoint, setDroneSetpoint] = useState(Array.apply(null, Array(NUM_DRONES)).map(() => (["0", "0", "0"])))
  const [droneSetpointWithMotion, setDroneSetpointWithMotion] = useState([0, 0, 0])
  const [droneTrim, setDroneTrim] = useState(["0", "0", "0", "0"])

  const [motionPreset, setMotionPreset] = useState(["setpoint", "setpoint"])

  const [trajectoryPlanningMaxVel, setTrajectoryPlanningMaxVel] = useState(["1", "1", "1"])
  const [trajectoryPlanningMaxAccel, setTrajectoryPlanningMaxAccel] = useState(["1", "1", "1"])
  const [trajectoryPlanningMaxJerk, setTrajectoryPlanningMaxJerk] = useState(["0.5", "0.5", "0.5"])
  // const [trajectoryPlanningWaypoints, setTrajectoryPlanningWaypoints] = useState("[0.2,0.2,0.5,true],\n[-0.2,0.2,0.5,true],\n[-0.2,0.2,0.8,true],\n[-0.2,-0.2,0.8,true],\n[-0.2,-0.2,0.5,true],\n[0.2,-0.2,0.5,true],\n[0.2,-0.2,0.8,true],\n[0.2,0.2,0.8,true],\n[0.2,0.2,0.5,true]\n]")
  const [trajectoryPlanningWaypoints, setTrajectoryPlanningWaypoints] = useState("[\n[0.2,0.2,0.6,0,0,0.8,true],\n[-0.2,0.2,0.6,0.2,0.2,0.6,true],\n[-0.2,-0.2,0.5,0,0,0.4,true],\n[0.2,-0.2,0.5,-0.2,-0.2,0.6,true],\n[0.2,0.2,0.5,0,0,0.8,true]\n]")
  const [trajectoryPlanningSetpoints, setTrajectoryPlanningSetpoints] = useState<number[][][]>([])
  const [trajectoryPlanningRunStartTimestamp, setTrajectoryPlanningRunStartTimestamp] = useState(0)

  const updateCameraSettings: FormEventHandler = (e) => {
    e.preventDefault()
    socket.emit("update-camera-settings", {
      exposure,
      gain,
    })
  }

  const capturePointsForPose = async (startOrStop: string) => {
    if (startOrStop === "start") {
      setCapturedPointsForPose("")
    }
    socket.emit("capture-points", { startOrStop })
  }

  useEffect(() => {
    socket.on("image-points", (data) => {
      console.log(data);
      setCapturedPointsForPose(`${capturedPointsForPose}${JSON.stringify(data)},`)
    })

    return () => {
      socket.off("image-points")
    }
  }, [capturedPointsForPose])

  useEffect(() => {
    let count = 0
    socket.emit("arm-drone", { droneArmed, count, currentDroneIndex })
    const pingInterval = setInterval(() => {
      count += 1
      socket.emit("arm-drone", { droneArmed, count, currentDroneIndex })
    }, 500)

    return () => {
      clearInterval(pingInterval)
    }
  }, [droneArmed])

  useEffect(() => {
    for (let droneIndex = 0; droneIndex < NUM_DRONES; droneIndex++) {
      socket.emit("set-drone-pid", { dronePID, droneIndex })
    }
  }, [dronePID])

  useEffect(() => {
    socket.emit("set-drone-trim", { droneTrim, droneIndex: currentDroneIndex })
  }, [droneTrim])

  useEffect(() => {
    let timestamp = Date.now() / 1000
    let motionIntervals: NodeJS.Timer[] = []

    for (let droneIndex = 0; droneIndex < NUM_DRONES; droneIndex++) {
      if (motionPreset[droneIndex] !== "setpoint") {
        motionIntervals.push(setInterval(() => {
          timestamp = Date.now() / 1000
          let tempDroneSetpoint = [] as number[]

          switch (motionPreset[droneIndex]) {
            case "none": {
              break;
            }

            case "circle": {
              const radius = 0.3
              const period = 10

              let tempDroneSetpoint: number[] = []

              // drones doing circles demo
              switch (droneIndex) {
                case 0: {
                  tempDroneSetpoint = [
                    radius * Math.cos(timestamp * 2 * Math.PI / period),
                    radius * Math.sin(timestamp * 2 * Math.PI / period),
                    parseFloat(droneSetpoint[droneIndex][2])
                  ]
                  break;
                }

                case 1: {
                  tempDroneSetpoint = [
                    0,
                    radius * Math.cos(timestamp * 2 * Math.PI / period),
                    parseFloat(droneSetpoint[droneIndex][2]) + radius * Math.sin(timestamp * 2 * Math.PI / period)
                  ]
                  break;
                }
              }
              tempDroneSetpoint.map(x => x.toFixed(3))
              socket.emit("set-drone-setpoint", { "droneSetpoint": tempDroneSetpoint, droneIndex })
              break;
            }

            case "square": {
              const size = 0.2
              const period = 20
              let offset = [0, 0]
              switch (Math.floor((timestamp * 4) / period) % 4) {
                case 0:
                  offset = [1, 1]
                  break
                case 1:
                  offset = [1, -1]
                  break
                case 2:
                  offset = [-1, -1]
                  break
                case 3:
                  offset = [-1, 1]
                  break
              }

              tempDroneSetpoint = [
                parseFloat(droneSetpoint[droneIndex][0]) + (offset[0] * size),
                parseFloat(droneSetpoint[droneIndex][1]) + (offset[1] * size),
                parseFloat(droneSetpoint[droneIndex][2])
              ]
              tempDroneSetpoint.map(x => x.toFixed(3))
              socket.emit("set-drone-setpoint", { "droneSetpoint": tempDroneSetpoint, droneIndex })
              break;
            }

            case "plannedTrajectory": {
              const index = Math.floor((timestamp - trajectoryPlanningRunStartTimestamp) / TRAJECTORY_PLANNING_TIMESTEP)
              if (index < trajectoryPlanningSetpoints.length) {
                tempDroneSetpoint = trajectoryPlanningSetpoints[droneIndex][index]
                tempDroneSetpoint.map(x => x.toFixed(3))
                socket.emit("set-drone-setpoint", { "droneSetpoint": tempDroneSetpoint, droneIndex })
              }
              else {
                let newMotionPreset = motionPreset.slice()
                newMotionPreset[droneIndex] = "setpoint"
                setMotionPreset(newMotionPreset)
              }
              break;
            }

            default:
              break;
          }

          if (droneIndex === currentDroneIndex) {
            setDroneSetpointWithMotion(tempDroneSetpoint)
          }
        }, TRAJECTORY_PLANNING_TIMESTEP * 1000))
      }
      else {
        if (droneIndex === currentDroneIndex) {
          setDroneSetpointWithMotion(droneSetpoint[droneIndex].map(x => parseFloat(x)))
        }
        socket.emit("set-drone-setpoint", { "droneSetpoint": droneSetpoint[droneIndex], droneIndex })
      }
    }

    return () => {
      motionIntervals.forEach(motionInterval => {
        clearInterval(motionInterval)
      })
    }
  }, [motionPreset, droneSetpoint, trajectoryPlanningRunStartTimestamp])

  useEffect(() => {
    socket.on("to-world-coords-matrix", (data) => {
      setToWorldCoordsMatrix(data["to_world_coords_matrix"])
      setObjectPointCount(objectPointCount + 1)
    })

    return () => {
      socket.off("to-world-coords-matrix")
    }
  }, [objectPointCount])

  useEffect(() => {
    socket.on("object-points", (data) => {
      console.log(data);
      // console.log(data["object_points"][0])
      objectPoints.current.push(data["object_points"])
      if (data["filtered_objects"].length != 0) {
        filteredObjects.current.push(data["filtered_objects"])
      }
      objectPointErrors.current.push(data["errors"])
      objects.current.push(data["objects"])
      droneSetpointHistory.current.push(droneSetpointWithMotion)
      setObjectPointCount(objectPointCount + 1)
    })

    return () => {
      socket.off("object-points")
    }
  }, [objectPointCount])

  useEffect(() => {
    // Socket listener for triangle points
    socket.on("triangle-points", (data) => {
      const newPoints = data["triangle_points"];
      if (newPoints && newPoints.length === 3) {
        trianglePointsRef.current = newPoints; // Update the ref with new points
      } else {
        console.warn("Invalid triangle points data received");
      }
    });

    return () => {
      socket.off("triangle-points"); // Cleanup on unmount
    };
  }, []);

  useEffect(() => {
    socket.on("camera-pose", data => {
      console.log(data["camera_poses"])
      setCameraPoses(data["camera_poses"])
    })

    return () => {
      socket.off("camera-pose")
    }
  }, [])

  useEffect(() => {
    socket.on("fps", data => {
      setFps(data["fps"])
    })

    return () => {
      socket.off("fps")
    }
  }, [])

  const planTrajectory = async (waypoints: object, maxVel: number[], maxAccel: number[], maxJerk: number[], timestep: number) => {
    const location = window.location.hostname;
    const settings = {
      method: 'POST',
      headers: {
        Accept: 'application/json',
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        waypoints,
        maxVel,
        maxAccel,
        maxJerk,
        timestep
      })
    };
    const fetchResponse = await fetch(`http://localhost:3001/api/trajectory-planning`, settings);
    const data = await fetchResponse.json();

    return data.setpoints
  }

  const wait = async (ms: number) => new Promise(r => setTimeout(r, ms))

  const moveToPos = async (pos: number[], droneIndex: number) => {
    console.log(filteredObjects.current[filteredObjects.current.length - 1][droneIndex])
    const waypoints = [
      filteredObjects.current[filteredObjects.current.length - 1][droneIndex]["pos"].concat([true]),
      pos.concat([true])
    ]
    const setpoints = await planTrajectory(
      waypoints,
      trajectoryPlanningMaxVel.map(x => parseFloat(x)),
      trajectoryPlanningMaxAccel.map(x => parseFloat(x)),
      trajectoryPlanningMaxJerk.map(x => parseFloat(x)),
      TRAJECTORY_PLANNING_TIMESTEP
    )

    for await (const [i, setpoint] of setpoints.entries()) {
      setpoint.map(x => x.toFixed(3))
      socket.emit("set-drone-setpoint", { "droneSetpoint": setpoint, droneIndex })
      setDroneSetpointWithMotion(setpoint)

      // if (land && i > 0.75*setpoints.length && filteredObjects.current[filteredObjects.current.length-1]["vel"][2] >= -0.2) {
      //   setDroneArmed(false)
      // }

      await wait(TRAJECTORY_PLANNING_TIMESTEP * 1000)
    }
  }

  const calculateCameraPose = async (cameraPoints: Array<Array<Array<number>>>) => {
    socket.emit("calculate-camera-pose", { cameraPoints })
  }

  const calculateCameraPoseTriangle = async (cameraPoints: Array<Array<Array<number>>>) => {
    socket.emit("calculate-camera-pose-triangle", { cameraPoints })
  }

  const isValidJson = (str: string) => {
    try {
      const o = JSON.parse(str);
      if (o && typeof o === "object") {
        return true;
      }
    } catch (e) { }
    return false;
  }

  const startLiveMocap = (startOrStop: string) => {
    socket.emit("triangulate-points", { startOrStop, cameraPoses, toWorldCoordsMatrix })
  }

  return (
    <Container fluid>
      <Row className="mt-3 mb-3 flex-nowrap" style={{ alignItems: 'center' }}>
        <Col className="ms-4" style={{ width: 'fit-content' }} md="auto">
          <h2>MoCap</h2>
        </Col>
        <Col>
          <Toolbar />
        </Col>
      </Row>
      <Row>
        <Col>
          <Card className='shadow-sm p-3'>
            <Row>
              <Col xs="auto">
                <h4>Camera Stream</h4>
              </Col>
              <Col>
                <Button
                  size='sm'
                  className='me-3'
                  variant={cameraStreamRunning ? "outline-danger" : "outline-primary"}
                  onClick={() => {
                    setCameraStreamRunning(!cameraStreamRunning);
                  }}
                >
                  {cameraStreamRunning ? "Stop" : "Start"}
                </Button>
                FPS: {fps}
              </Col>
            </Row>
            <Row className='mt-2 mb-1' style={{ height: "512px" }}>
              <Col>
                <img src={cameraStreamRunning ? "http://localhost:3001/api/camera-stream" : ""} />
              </Col>
            </Row>
          </Card>
        </Col>
      </Row>
      <Row className='pt-3'>
        <Col xs={4}>
          <Card className='shadow-sm p-3 h-100'>
            <Row>
              <Col xs="auto">
                <h4>Camera Settings</h4>
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col xs="4">
                <Form onChange={updateCameraSettings} className='ps-3'>
                  <Form.Group className="mb-1">
                    <Form.Label>Exposure: {exposure}</Form.Label>
                    <Form.Range value={exposure} onChange={(event) => setExposure(parseFloat(event.target.value))} />
                  </Form.Group>
                  <Form.Group className="mb-1">
                    <Form.Label>Gain: {gain}</Form.Label>
                    <Form.Range value={gain} onChange={(event) => setGain(parseFloat(event.target.value))} />
                  </Form.Group>
                </Form>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4>Live Triangulation</h4>
              </Col>
              <Col>
                <Button
                  size='sm'
                  variant={isTriangulatingPoints ? "outline-danger" : "outline-primary"}
                  disabled={!cameraStreamRunning}
                  onClick={() => {
                    if (!isTriangulatingPoints) {
                      objectPoints.current = []
                      objectPointErrors.current = []
                      objects.current = []
                      filteredObjects.current = []
                      droneSetpointHistory.current = []
                    }
                    setIsTriangulatingPoints(!isTriangulatingPoints);
                    startLiveMocap(isTriangulatingPoints ? "stop" : "start");
                  }
                  }>
                  {isTriangulatingPoints ? "Stop" : "Start"}
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4>Locate Objects</h4>
              </Col>
              <Col>
                <Button
                  size='sm'
                  variant={isLocatingObjects ? "outline-danger" : "outline-primary"}
                  disabled={!cameraStreamRunning}
                  onClick={() => {
                    setIsLocatingObjects(!isLocatingObjects);
                    socket.emit("locate-objects", { startOrStop: isLocatingObjects ? "stop" : "start" })
                  }
                  }>
                  {isLocatingObjects ? "Stop" : "Start"}
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4>Set Scale Using Points</h4>
              </Col>
              <Col>
                <Button
                  size='sm'
                  variant="outline-primary"
                  disabled={!isTriangulatingPoints && objectPoints.current.length == 0}
                  onClick={() => {
                    socket.emit("determine-scale", { objectPoints: objectPoints.current, cameraPoses: cameraPoses })
                  }
                  }>
                  Run
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4>Print Average Point</h4>
              </Col>
              <Col>
                <Button
                  size='sm'
                  variant="outline-primary"
                  disabled={!isTriangulatingPoints && objectPoints.current.length == 0}
                  onClick={() => {
                    socket.emit("determine-average", { objectPoints: objectPoints.current, cameraPoses: cameraPoses })
                  }
                  }>
                  Run
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4><del>Add Beacons</del></h4>
              </Col>
              <Col>
                <Button
                  size='sm'
                  variant="outline-primary"
                  onClick={() => {
                    socket.emit("add-beacons", {})
                  }
                  }>
                  Run
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4><del>Save Points</del></h4>
              </Col>
              <Col>
                <Button
                  size='sm'
                  variant="outline-primary"
                  onClick={() => {
                    socket.emit("save-points", { objectPoints: objectPoints.current, saveNumber: saveNumber, objectPointErrors: objectPointErrors.current})
                  }
                  }>
                  Run
                </Button>
              </Col>
              <Col xs={4} className='pt-2'>
                Number:
              </Col>
              <Col>
                <Form.Control
                  value={saveNumber}
                  onChange={(event) => setSaveNumber(parseFloat(event.target.value))}
                />
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4><del>Load Points</del></h4>
              </Col>
              <Col>
                <Button
                  size='sm'
                  variant="outline-primary"
                  onClick={() => {
                    socket.emit("load-points", { objectPoints: objectPoints.current, saveNumber: saveNumber, objectPointErrors: objectPointErrors.current})
                  }
                  }>
                  Run
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4><del>Test Diff</del></h4>
              </Col>
              <Col>
                <Button
                  size='sm'
                  variant="outline-primary"
                  onClick={() => {
                    socket.emit("test-diff", {})
                  }
                  }>
                  Run
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4>Reset Simulation Gyzmos</h4>
              </Col>
              <Col>
                <Button
                  size='sm'
                  variant="outline-primary"
                  onClick={() => {
                    socket.emit("reset-simulation-gyzmos", {})
                  }
                  }>
                  Run
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4>Load Real Poses from Simulation</h4>
              </Col>
              <Col>
                <Button
                  size='sm'
                  variant="outline-primary"
                  onClick={() => {
                    socket.emit("load-simulation-poses", {})
                  }
                  }>
                  Run
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4>Acquire Floor</h4>
              </Col>
              <Col>
                <Button
                  size='sm'
                  variant="outline-primary"
                  disabled={!isTriangulatingPoints && objectPoints.current.length == 0}
                  onClick={() => {
                    socket.emit("acquire-floor", { objectPoints: objectPoints.current })
                  }
                  }>
                  Run
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4>Set Origin</h4>
              </Col>
              <Col>
                <Button
                  size='sm'
                  variant="outline-primary"
                  disabled={!isTriangulatingPoints && objectPoints.current.length == 0}
                  onClick={() => {
                    socket.emit("set-origin", { objectPoint: objectPoints.current[0][0], toWorldCoordsMatrix })
                  }
                  }>
                  Run
                </Button>
              </Col>
            </Row>
            <Row>
              <Col xs="auto">
                <h4>Collect points for camera pose calibration</h4>
              </Col>
              <Col>
                <Tooltip id="collect-points-for-pose-button-tooltip" />
                <a data-tooltip-hidden={cameraStreamRunning} data-tooltip-variant='error' data-tooltip-id='collect-points-for-pose-button-tooltip' data-tooltip-content="Start camera stream first">
                  <Button
                    size='sm'
                    variant={capturingPointsForPose ? "outline-danger" : "outline-primary"}
                    disabled={!cameraStreamRunning}
                    onClick={() => {
                      setCapturingPointsForPose(!capturingPointsForPose);
                      capturePointsForPose(capturingPointsForPose ? "stop" : "start");
                    }
                    }>
                    {capturingPointsForPose ? "Stop" : "Start"}
                  </Button>
                </a>
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col>
                <Button
                  size='sm'
                  className='float-end'
                  variant="outline-primary"
                  disabled={!(isValidJson(`[${capturedPointsForPose.slice(0, -1)}]`) && JSON.parse(`[${capturedPointsForPose.slice(0, -1)}]`).length !== 0)}
                  onClick={() => {
                    calculateCameraPose(JSON.parse(`[${capturedPointsForPose.slice(0, -1)}]`))
                  }}>
                  Calculate Camera Pose with {isValidJson(`[${capturedPointsForPose.slice(0, -1)}]`) ? JSON.parse(`[${capturedPointsForPose.slice(0, -1)}]`).length : 0} points
                </Button>
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col>
                <Button
                  size='sm'
                  className='float-end'
                  variant="outline-primary"
                  disabled={!(isValidJson(`[${capturedPointsForPose.slice(0, -1)}]`) && JSON.parse(`[${capturedPointsForPose.slice(0, -1)}]`).length !== 0)}
                  onClick={() => {
                    calculateCameraPoseTriangle(JSON.parse(`[${capturedPointsForPose.slice(0, -1)}]`))
                  }}>
                  Calculate Camera Pose using Triangle with {isValidJson(`[${capturedPointsForPose.slice(0, -1)}]`) ? JSON.parse(`[${capturedPointsForPose.slice(0, -1)}]`).length : 0} points
                </Button>
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col xs={4} className='pt-2'>
                Camera Poses:
              </Col>
              <Col>
                <Form.Control
                  value={JSON.stringify(cameraPoses)}
                  onChange={(event) => setCameraPoses(JSON.parse(event.target.value))}
                />
              </Col>
            </Row>
            <Row>
              <Col xs={4} className='pt-2'>
                To World Matrix:
              </Col>
              <Col>
                <Form.Control
                  value={JSON.stringify(toWorldCoordsMatrix)}
                  onChange={(event) => setToWorldCoordsMatrix(JSON.parse(event.target.value))}
                />
              </Col>
            </Row>

            <CameraRotationControl></CameraRotationControl>
          </Card>
        </Col>
        <Col xs={4}>
          <Card className='shadow-sm p-3 h-100'>
            <Row>
              <Col>
                <h4>Generate Trajectory</h4>
              </Col>
            </Row>
            <Row className='pt-1'>
              <Col xs={{ offset: 3 }} className='text-center'>
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
              <Col xs={3} className='pt-2 text-end'>
                Max Vel
              </Col>
              <Col>
                <Form.Control
                  value={trajectoryPlanningMaxVel[0]}
                  onChange={(event) => {
                    let newTrajectoryPlanningMaxVel = trajectoryPlanningMaxVel.slice()
                    newTrajectoryPlanningMaxVel[0] = event.target.value
                    setTrajectoryPlanningMaxVel(newTrajectoryPlanningMaxVel)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={trajectoryPlanningMaxVel[1]}
                  onChange={(event) => {
                    let newTrajectoryPlanningMaxVel = trajectoryPlanningMaxVel.slice()
                    newTrajectoryPlanningMaxVel[1] = event.target.value
                    setTrajectoryPlanningMaxVel(newTrajectoryPlanningMaxVel)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={trajectoryPlanningMaxVel[2]}
                  onChange={(event) => {
                    let newTrajectoryPlanningMaxVel = trajectoryPlanningMaxVel.slice()
                    newTrajectoryPlanningMaxVel[2] = event.target.value
                    setTrajectoryPlanningMaxVel(newTrajectoryPlanningMaxVel)
                  }}
                />
              </Col>
            </Row>
            <Row className='pt-2'>
              <Col xs={3} className='pt-2 text-end'>
                Max Accel
              </Col>
              <Col>
                <Form.Control
                  value={trajectoryPlanningMaxAccel[0]}
                  onChange={(event) => {
                    let newTrajectoryPlanningMaxAccel = trajectoryPlanningMaxAccel.slice()
                    newTrajectoryPlanningMaxAccel[0] = event.target.value
                    setTrajectoryPlanningMaxAccel(newTrajectoryPlanningMaxAccel)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={trajectoryPlanningMaxAccel[1]}
                  onChange={(event) => {
                    let newTrajectoryPlanningMaxAccel = trajectoryPlanningMaxAccel.slice()
                    newTrajectoryPlanningMaxAccel[1] = event.target.value
                    setTrajectoryPlanningMaxAccel(newTrajectoryPlanningMaxAccel)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={trajectoryPlanningMaxAccel[2]}
                  onChange={(event) => {
                    let newTrajectoryPlanningMaxAccel = trajectoryPlanningMaxAccel.slice()
                    newTrajectoryPlanningMaxAccel[2] = event.target.value
                    setTrajectoryPlanningMaxAccel(newTrajectoryPlanningMaxAccel)
                  }}
                />
              </Col>
            </Row>
            <Row className='pt-2'>
              <Col xs={3} className='pt-2 text-end'>
                Max Jerk
              </Col>
              <Col>
                <Form.Control
                  value={trajectoryPlanningMaxJerk[0]}
                  onChange={(event) => {
                    let newTrajectoryPlanningMaxJerk = trajectoryPlanningMaxJerk.slice()
                    newTrajectoryPlanningMaxJerk[0] = event.target.value
                    setTrajectoryPlanningMaxJerk(newTrajectoryPlanningMaxJerk)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={trajectoryPlanningMaxJerk[1]}
                  onChange={(event) => {
                    let newTrajectoryPlanningMaxJerk = trajectoryPlanningMaxJerk.slice()
                    newTrajectoryPlanningMaxJerk[1] = event.target.value
                    setTrajectoryPlanningMaxJerk(newTrajectoryPlanningMaxJerk)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={trajectoryPlanningMaxJerk[2]}
                  onChange={(event) => {
                    let newTrajectoryPlanningMaxJerk = trajectoryPlanningMaxJerk.slice()
                    newTrajectoryPlanningMaxJerk[2] = event.target.value
                    setTrajectoryPlanningMaxJerk(newTrajectoryPlanningMaxJerk)
                  }}
                />
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col>
                Waypoints <code>[drone index, x, y, z, stop at waypoint]</code>
              </Col>
            </Row>
            <Row className='pt-1'>
              <Col>
                <Form.Control
                  as="textarea"
                  rows={5}
                  value={trajectoryPlanningWaypoints}
                  onChange={(event) => setTrajectoryPlanningWaypoints(event.target.value)}
                />
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col>
                <Button
                  size='sm'
                  className='float-end'
                  variant={droneArmed ? "outline-danger" : "outline-primary"}
                  onClick={async () => {
                    setMotionPreset(new Array(NUM_DRONES).fill("none"))
                    const initPos = JSON.parse(trajectoryPlanningWaypoints)[0].slice(0, 3)
                    await Promise.all(Array.from(Array(NUM_DRONES).keys()).map(async (droneIndex) => {
                      await moveToPos(initPos, droneIndex)
                    }))
                    setTrajectoryPlanningRunStartTimestamp(Date.now() / 1000)
                    setMotionPreset(new Array(NUM_DRONES).fill("plannedTrajectory"))
                  }}
                >
                  Run
                </Button>
                <Button
                  size='sm'
                  className='float-end me-2'
                  variant={droneArmed ? "outline-danger" : "outline-primary"}
                  onClick={async () => {
                    const tempSetpoints = await planTrajectory(
                      JSON.parse(trajectoryPlanningWaypoints),
                      trajectoryPlanningMaxVel.map(x => parseFloat(x)),
                      trajectoryPlanningMaxAccel.map(x => parseFloat(x)),
                      trajectoryPlanningMaxJerk.map(x => parseFloat(x)),
                      TRAJECTORY_PLANNING_TIMESTEP
                    )
                    setTrajectoryPlanningSetpoints(tempSetpoints)
                  }}
                >
                  Preview
                </Button>
              </Col>
            </Row>
          </Card>
        </Col>
        <Col xs={4}>
          <Card className='shadow-sm p-3 h-100'>
            <Row>
              <Col xs="auto">
                <h4>Control Drone</h4>
              </Col>
              <Col xs="3">
                <Form.Select value={currentDroneIndex} onChange={(e) => setCurrentDroneIndex(parseInt(e.target.value))} size='sm'>
                  <option value="0">Drone 0</option>
                  <option value="1">Drone 1</option>
                </Form.Select>
              </Col>
            </Row>
            {Array.from(Array(NUM_DRONES).keys()).map((droneIndex) => (
              <>
                <Row className='pt-4'>
                  <Col xs="3">
                    <h5>Drone {droneIndex}</h5>
                  </Col>
                  <Col className='text-center'>
                    X
                  </Col>
                  <Col className='text-center'>
                    Y
                  </Col>
                  <Col className='text-center'>
                    Z
                  </Col>
                </Row>
                <Row>
                  <Col xs={3} className='pt-2'>
                    Setpoint
                  </Col>
                  <Col>
                    <Form.Control
                      value={droneSetpoint[droneIndex][0]}
                      onChange={(event) => {
                        let newDroneSetpoint = droneSetpoint.slice()
                        newDroneSetpoint[droneIndex][0] = event.target.value
                        setDroneSetpoint(newDroneSetpoint)
                      }}
                    />
                  </Col>
                  <Col>
                    <Form.Control
                      value={droneSetpoint[droneIndex][1]}
                      onChange={(event) => {
                        let newDroneSetpoint = droneSetpoint.slice()
                        newDroneSetpoint[droneIndex][1] = event.target.value
                        setDroneSetpoint(newDroneSetpoint)
                      }}
                    />
                  </Col>
                  <Col>
                    <Form.Control
                      value={droneSetpoint[droneIndex][2]}
                      onChange={(event) => {
                        let newDroneSetpoint = droneSetpoint.slice()
                        newDroneSetpoint[droneIndex][2] = event.target.value
                        setDroneSetpoint(newDroneSetpoint)
                      }}
                    />
                  </Col>
                </Row>
                <Row className='pt-3'>
                  <Col>
                    <Button
                      size='sm'
                      variant={droneArmed[droneIndex] ? "outline-danger" : "outline-primary"}
                      disabled={!isTriangulatingPoints}
                      onClick={() => {
                        let newDroneArmed = droneArmed.slice()
                        newDroneArmed[droneIndex] = !newDroneArmed[droneIndex]
                        setDroneArmed(newDroneArmed);
                      }
                      }>
                      {droneArmed[droneIndex] ? "Disarm" : "Arm"}
                    </Button>
                  </Col>
                  <Col>
                    <Button
                      size='sm'
                      onClick={() => {
                        let newMotionPreset = motionPreset.slice()
                        newMotionPreset[droneIndex] = "setpoint"
                        setMotionPreset(newMotionPreset);
                      }
                      }>
                      Setpoint
                    </Button>
                  </Col>
                  <Col>
                    <Button
                      size='sm'
                      onClick={() => {
                        let newMotionPreset = motionPreset.slice()
                        newMotionPreset[droneIndex] = "circle"
                        setMotionPreset(newMotionPreset);
                      }
                      }>
                      Circle
                    </Button>
                  </Col>
                  <Col>
                    <Button
                      size='sm'
                      onClick={() => {
                        let newMotionPreset = motionPreset.slice()
                        newMotionPreset[droneIndex] = "square"
                        setMotionPreset(newMotionPreset);
                      }
                      }>
                      Square
                    </Button>
                  </Col>
                  <Col>
                    <Button
                      size='sm'
                      onClick={async () => {
                        await moveToPos([0, 0, LAND_Z_HEIGHT], droneIndex)

                        let newDroneArmed = droneArmed.slice()
                        newDroneArmed[droneIndex] = false
                        setDroneArmed(newDroneArmed);

                        let newMotionPreset = motionPreset.slice()
                        newMotionPreset[droneIndex] = "setpoint"
                        setMotionPreset(newMotionPreset);
                      }
                      }>
                      Land
                    </Button>
                  </Col>
                </Row>
              </>
            ))}
            <Row className='pt-3'>
              <Col xs={{ offset: 2 }} className='text-center'>
                Pos P
              </Col>
              <Col className='text-center'>
                Pos I
              </Col>
              <Col className='text-center'>
                Pos D
              </Col>
            </Row>
            <Row className='pt-2'>
              <Col xs={2} className='pt-2 text-end'>
                XY
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[0]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[0] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[1]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[1] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[2]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[2] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col xs={2} className='pt-2 text-end'>
                Z
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[3]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[3] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[4]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[4] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[5]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[5] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col xs={2} className='pt-2 text-end'>
                YAW
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[6]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[6] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[7]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[7] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[8]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[8] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col xs={{ offset: 2 }} className='text-center'>
                Vel P
              </Col>
              <Col className='text-center'>
                Vel I
              </Col>
              <Col className='text-center'>
                Vel D
              </Col>
            </Row>
            <Row className='pt-2'>
              <Col xs={2} className='pt-2 text-end'>
                XY
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[9]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[9] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[10]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[10] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[11]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[11] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col xs={2} className='pt-2 text-end'>
                Z
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[12]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[12] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[13]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[13] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[14]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[14] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
            </Row>
            <Row>
              <Col>
                <Row className="mt-3 mb-1">
                  <Col xs={4}>
                    <Form.Label>X Trim: {droneTrim[0]}</Form.Label>
                  </Col>
                  <Col>
                    <Form.Range value={droneTrim[0]} min={-800} max={800} onChange={(event) => {
                      let newDroneTrim = droneTrim.slice()
                      newDroneTrim[0] = event.target.value
                      setDroneTrim(newDroneTrim)
                    }} />
                  </Col>
                </Row>
                <Row className="mb-1">
                  <Col xs={4}>
                    <Form.Label>Y Trim: {droneTrim[1]}</Form.Label>
                  </Col>
                  <Col>
                    <Form.Range value={droneTrim[1]} min={-800} max={800} onChange={(event) => {
                      let newDroneTrim = droneTrim.slice()
                      newDroneTrim[1] = event.target.value
                      setDroneTrim(newDroneTrim)
                    }} />
                  </Col>
                </Row>
                <Row className="mb-1">
                  <Col xs={4}>
                    <Form.Label>Z Trim: {droneTrim[2]}</Form.Label>
                  </Col>
                  <Col>
                    <Form.Range value={droneTrim[2]} min={-800} max={800} onChange={(event) => {
                      let newDroneTrim = droneTrim.slice()
                      newDroneTrim[2] = event.target.value
                      setDroneTrim(newDroneTrim)
                    }} />
                  </Col>
                </Row>
                <Row className="mb-1">
                  <Col xs={4}>
                    <Form.Label>Yaw Trim: {droneTrim[3]}</Form.Label>
                  </Col>
                  <Col>
                    <Form.Range value={droneTrim[3]} min={-800} max={800} onChange={(event) => {
                      let newDroneTrim = droneTrim.slice()
                      newDroneTrim[3] = event.target.value
                      setDroneTrim(newDroneTrim)
                    }} />
                  </Col>
                </Row>
              </Col>
            </Row>
            <Row className='pt-3'>
              <Col className='pt-2'>
                Ground Effect Coef.
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[15]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[15] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
              <Col className='pt-2'>
                Ground Effect Offset
              </Col>
              <Col>
                <Form.Control
                  value={dronePID[16]}
                  onChange={(event) => {
                    let newDronePID = dronePID.slice()
                    newDronePID[16] = event.target.value
                    setDronePID(newDronePID)
                  }}
                />
              </Col>
            </Row>
          </Card>
        </Col>
      </Row>
      {/* <Row className='pt-3'>
        <Col>
          <Card className='shadow-sm p-3'>
            <Chart filteredObjectsRef={filteredObjects} droneSetpointHistoryRef={droneSetpointHistory} objectPointCount={objectPointCount} dronePID={dronePID.map(x => parseFloat(x))} droneArmed={droneArmed} currentDroneIndex={currentDroneIndex} />
          </Card>
        </Col>
      </Row> */}
      <Row className='pt-3'>
        <Col>
          <Card className='shadow-sm p-3'>
            <Row>
              <Col xs="auto">
                {/* <h4>Scene Viewer {objectPointErrors.current.length !== 0 ? mean(objectPointErrors.current.flat()) : ""}</h4> */}
              </Col>
            </Row>
            <Row>
              <Col style={{ height: "1000px" }}>
                <Canvas camera={{ fov: 50, position: [0, 0, 3] }}>
                  <ambientLight />
                  {cameraPoses.map(({ R, t }, i) => (
                    <CameraWireframe R={R} t={t} toWorldCoordsMatrix={toWorldCoordsMatrix} key={i} />
                  ))}
                  <Points objectPointsRef={objectPoints} objectPointErrorsRef={objectPointErrors} count={objectPointCount} />
                  <Objects filteredObjectsRef={filteredObjects} count={objectPointCount} />
                  <FilledTriangle trianglePointsRef={trianglePointsRef} />
                  <TrajectoryPlanningSetpoints trajectoryPlanningSetpoints={trajectoryPlanningSetpoints} NUM_DRONES={NUM_DRONES} />
                  <OrbitControls />
                  <axesHelper args={[0.2]} />
                  <gridHelper args={[4, 4 * 10]} />
                  <directionalLight />
                </Canvas>
              </Col>
            </Row>
          </Card>
        </Col>
      </Row>
    </Container>
  )
}
