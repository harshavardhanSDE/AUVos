import React, { useEffect, useState } from 'react'
import { Canvas } from '@react-three/fiber'
import { useGLTF, OrbitControls } from '@react-three/drei'

function Model({ modelData }: { modelData: string | null }) {
    if (!modelData) return null

    const gltfData = new Blob([Buffer.from(modelData, 'base64')], { type: 'model/gltf+json' })
    const url = URL.createObjectURL(gltfData)
    const { scene } = useGLTF(url)

    return <primitive object={scene} scale={1} />
}

export default function App() {
    const [modelData, setModelData] = useState<string | null>(null)

    useEffect(() => {
        const ws = new WebSocket('ws://localhost:8080')

        ws.onopen = () => {
            console.log('Connected to WebSocket')
            ws.send('steampunk_underwater_explorer') // Request the model
        }

        ws.onmessage = (event) => {
            const { data } = JSON.parse(event.data)
            if (data) setModelData(data)
        }

        ws.onclose = () => console.log('Disconnected from WebSocket')

        return () => ws.close()
    }, [])

    return (
        <Canvas>
            <ambientLight intensity={0.5} />
            <directionalLight position={[5, 5, 5]} />
            <Model modelData={modelData} />
            <OrbitControls />
        </Canvas>
    )
}
