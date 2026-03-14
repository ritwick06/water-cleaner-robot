import React, { useEffect, useRef, useState } from "react";
import L, { Map as LeafletMap, Polygon } from "leaflet";

type Telemetry = {
  t: number;
  lat: number;
  lon: number;
  heading_deg: number;
  speed_mps: number;
  mode: number;
};

export const App: React.FC = () => {
  const mapRef = useRef<LeafletMap | null>(null);
  const markerRef = useRef<L.Marker | null>(null);
  const polygonRef = useRef<Polygon | null>(null);
  const [telemetry, setTelemetry] = useState<Telemetry | null>(null);
  const [polygon, setPolygon] = useState<{ lat: number; lon: number }[]>([]);

  useEffect(() => {
    if (!mapRef.current) {
      const map = L.map("map", {
        zoomControl: false,
      }).setView([13.0, 80.0], 16);

      L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
        attribution: "&copy; OpenStreetMap contributors",
      }).addTo(map);

      map.on("click", (e: L.LeafletMouseEvent) => {
        const ll = e.latlng;
        setPolygon((prev) => [...prev, { lat: ll.lat, lon: ll.lng }]);
      });

      mapRef.current = map;
    }

    const map = mapRef.current!;
    if (!markerRef.current) {
      const icon = L.divIcon({
        className: "",
        html:
          '<div style="width:14px;height:14px;border-radius:9999px;background:#38bdf8;border:2px solid rgba(15,23,42,0.9);box-shadow:0 0 12px rgba(56,189,248,0.8);"></div>',
      });
      markerRef.current = L.marker([13.0, 80.0], { icon }).addTo(map);
    }
  }, []);

  useEffect(() => {
    const proto = window.location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(`${proto}://${window.location.host}/ws/telemetry`);

    ws.onmessage = (evt) => {
      const data = JSON.parse(evt.data) as Telemetry;
      setTelemetry(data);
      if (mapRef.current && markerRef.current) {
        markerRef.current.setLatLng([data.lat, data.lon]);
      }
    };

    return () => ws.close();
  }, []);

  useEffect(() => {
    if (!mapRef.current) return;
    if (polygon.length < 2) {
      if (polygonRef.current) {
        polygonRef.current.remove();
        polygonRef.current = null;
      }
      return;
    }
    const latlngs = polygon.map((p) => [p.lat, p.lon]) as [number, number][];
    if (!polygonRef.current) {
      polygonRef.current = L.polygon(latlngs, {
        color: "#38bdf8",
        weight: 2,
        fillColor: "#38bdf8",
        fillOpacity: 0.15,
      }).addTo(mapRef.current);
    } else {
      polygonRef.current.setLatLngs(latlngs);
    }
  }, [polygon]);

  const handleStartMission = async () => {
    if (polygon.length < 3) return;
    await fetch("/mission", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ polygon }),
    });
  };

  return (
    <div
      style={{
        display: "grid",
        gridTemplateColumns: "2fr 1fr",
        gap: "1.5rem",
        padding: "1.5rem",
        height: "100%",
        boxSizing: "border-box",
      }}
    >
      <div
        id="map"
        style={{
          width: "100%",
          height: "100%",
          borderRadius: "1.25rem",
          border: "1px solid rgba(148,163,184,0.35)",
          boxShadow: "0 18px 40px rgba(15,23,42,0.9)",
          overflow: "hidden",
        }}
      />
      <div
        style={{
          display: "flex",
          flexDirection: "column",
          gap: "1rem",
        }}
      >
        <div
          style={{
            padding: "1.25rem",
            borderRadius: "1.25rem",
            background:
              "linear-gradient(145deg, rgba(15,23,42,0.92), rgba(15,23,42,0.7))",
            border: "1px solid rgba(148,163,184,0.4)",
            backdropFilter: "blur(24px)",
            boxShadow:
              "0 22px 60px rgba(15,23,42,0.95), 0 0 0 1px rgba(148,163,184,0.1)",
          }}
        >
          <h1
            style={{
              margin: 0,
              fontSize: "1.4rem",
              fontWeight: 600,
              letterSpacing: "0.04em",
              textTransform: "uppercase",
              color: "#e5e7eb",
            }}
          >
            AWSCR Mission Console
          </h1>
          <p
            style={{
              marginTop: "0.35rem",
              fontSize: "0.85rem",
              color: "#9ca3af",
            }}
          >
            Real-time SIL telemetry over WebSockets
          </p>
        </div>

        <div
          style={{
            padding: "1.25rem",
            borderRadius: "1.25rem",
            background: "rgba(15,23,42,0.85)",
            border: "1px solid rgba(148,163,184,0.35)",
            backdropFilter: "blur(20px)",
            display: "grid",
            gridTemplateColumns: "repeat(2, minmax(0, 1fr))",
            gap: "1rem",
          }}
        >
          <MetricCard
            label="Latitude"
            value={telemetry ? telemetry.lat.toFixed(6) : "--"}
          />
          <MetricCard
            label="Longitude"
            value={telemetry ? telemetry.lon.toFixed(6) : "--"}
          />
          <MetricCard
            label="Heading"
            value={
              telemetry ? `${telemetry.heading_deg.toFixed(1)} °` : "--"
            }
          />
          <MetricCard
            label="Speed"
            value={telemetry ? `${telemetry.speed_mps.toFixed(2)} m/s` : "--"}
          />
        </div>
        <button
          onClick={handleStartMission}
          style={{
            marginTop: "0.5rem",
            padding: "0.85rem 1.1rem",
            borderRadius: "9999px",
            border: "1px solid rgba(56,189,248,0.6)",
            background:
              "radial-gradient(circle at top left, rgba(56,189,248,0.22), rgba(15,23,42,0.95))",
            color: "#e5e7eb",
            fontSize: "0.9rem",
            fontWeight: 500,
            letterSpacing: "0.06em",
            textTransform: "uppercase",
            cursor: polygon.length >= 3 ? "pointer" : "not-allowed",
            opacity: polygon.length >= 3 ? 1 : 0.4,
          }}
        >
          {polygon.length >= 3 ? "Start Mission" : "Tap map to draw polygon"}
        </button>
      </div>
    </div>
  );
};

const MetricCard: React.FC<{ label: string; value: string }> = ({
  label,
  value,
}) => (
  <div
    style={{
      padding: "0.9rem 1rem",
      borderRadius: "1rem",
      border: "1px solid rgba(148,163,184,0.35)",
      background:
        "radial-gradient(circle at top left, rgba(56,189,248,0.12), transparent 55%), rgba(15,23,42,0.9)",
      boxShadow: "0 14px 35px rgba(15,23,42,0.8)",
    }}
  >
    <div
      style={{
        fontSize: "0.75rem",
        textTransform: "uppercase",
        letterSpacing: "0.12em",
        color: "#9ca3af",
        marginBottom: "0.35rem",
      }}
    >
      {label}
    </div>
    <div
      style={{
        fontSize: "1.1rem",
        fontWeight: 600,
        color: "#f9fafb",
      }}
    >
      {value}
    </div>
  </div>
);

