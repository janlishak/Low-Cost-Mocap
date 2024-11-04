import { BufferAttribute, BufferGeometry, Mesh, MeshBasicMaterial, DoubleSide } from "three";
import { MutableRefObject, useEffect, useRef } from "react";

export default function FilledTriangle({ trianglePointsRef }: { trianglePointsRef: MutableRefObject<number[][]> }) {
  const geometryRef = useRef<BufferGeometry>(new BufferGeometry());

  // Helper function to swap Y and Z coordinates
  const swapYZ = (point: number[]) => {
    return [point[0], point[2], point[1]]; // Swaps Y (index 1) and Z (index 2)
  };

  useEffect(() => {
    // Ensure there are exactly three points provided
    if (trianglePointsRef.current.length !== 3) {
      console.warn("trianglePointsRef should contain exactly three points.");
      // Emit a default triangle if needed, or return
      return;
    }

    // Update the vertices based on the provided triangle points with Y and Z swapped
    const vertices = new Float32Array([
      ...swapYZ(trianglePointsRef.current[0]),
      ...swapYZ(trianglePointsRef.current[1]),
      ...swapYZ(trianglePointsRef.current[2]),
    ]);

    // Update the geometry's position attribute
    geometryRef.current.setAttribute('position', new BufferAttribute(vertices, 3));
    geometryRef.current.attributes.position.needsUpdate = true; // Mark attribute as needing an update

  }, [trianglePointsRef.current]); // Re-run the effect whenever trianglePointsRef changes

  // Define the material for filling the triangle
  const material = new MeshBasicMaterial({ color: 0x00ff00, side: DoubleSide });

  return (
    <mesh geometry={geometryRef.current} material={material} />
  );
}
