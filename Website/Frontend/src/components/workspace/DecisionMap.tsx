"use client";

import "maplibre-gl/dist/maplibre-gl.css";

import maplibregl, {
  type Map,
  type Marker,
  type StyleSpecification,
} from "maplibre-gl";
import { useEffect, useRef } from "react";
import type { Asset } from "@/types/asset";

// TILE SOURCE PLACEHOLDER — CARTO dark raster, free, no API key required.
// Replace with confirmed production tile source before launch.
const DARK_RASTER_STYLE: StyleSpecification = {
  version: 8,
  sources: {
    cartoDark: {
      type: "raster",
      tiles: [
        "https://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png",
        "https://b.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png",
        "https://c.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png",
      ],
      tileSize: 256,
      attribution:
        '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> &copy; <a href="https://carto.com/attributions">CARTO</a>',
    },
  },
  layers: [
    {
      id: "carto-dark",
      type: "raster",
      source: "cartoDark",
      minzoom: 0,
      maxzoom: 20,
      paint: { "raster-opacity": 0.78 },
    },
  ],
};

// Asset marker tone — based on coverage score (how well the asset is understood)
// and availability status. Uses the edgeai-marker CSS class system from globals.css.
function assetMarkerTone(asset: Asset): string {
  if (asset.availabilityStatus !== "available") return "pending";
  if (asset.coverageScore >= 70) return "high";
  if (asset.coverageScore >= 45) return "medium";
  return "low";
}

// Label shown inside the marker circle.
// For assets with assessments passed in, shows the best feasibility score.
// Otherwise shows the coverage score (how well the asset is understood).
function markerLabel(asset: Asset, assessedScores: Record<string, number>): string {
  const score = assessedScores[asset.id];
  if (score !== undefined) return String(score);
  if (asset.availabilityStatus !== "available") return "·";
  return String(asset.coverageScore);
}

interface DecisionMapProps {
  assets: Asset[];
  selectedAsset: Asset | null;
  onSelectAsset: (asset: Asset) => void;
  // Optional: show feasibility scores on markers for already-assessed assets.
  // Record<assetId, feasibilityScore> — passed from the workbench.
  assessedScores?: Record<string, number>;
}

export function DecisionMap({
  assets,
  selectedAsset,
  onSelectAsset,
  assessedScores = {},
}: DecisionMapProps) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<Map | null>(null);

  useEffect(() => {
    if (!containerRef.current || mapRef.current) return;

    const map = new maplibregl.Map({
      container: containerRef.current,
      style: DARK_RASTER_STYLE,
      center: [31.2357, 30.0444], // Cairo
      zoom: 10.2,
      minZoom: 8.5,
      maxZoom: 17,
      attributionControl: false,
    });

    map.addControl(
      new maplibregl.NavigationControl({ showCompass: false }),
      "bottom-left"
    );
    map.addControl(new maplibregl.AttributionControl({ compact: true }));

    mapRef.current = map;

    return () => {
      map.remove();
      mapRef.current = null;
    };
  }, []);

  // Rebuild markers when assets or selection changes
  useEffect(() => {
    const map = mapRef.current;
    if (!map || assets.length === 0) return;

    const markers: Marker[] = assets.map((asset) => {
      const tone = assetMarkerTone(asset);
      const isSelected = selectedAsset?.id === asset.id;
      const isAssessed = asset.id in assessedScores;

      const el = document.createElement("button");
      el.type = "button";
      el.className = [
        "edgeai-marker",
        `edgeai-marker--${tone}`,
        isSelected ? "edgeai-marker--selected" : "",
        isAssessed ? "edgeai-marker--assessed" : "",
      ]
        .filter(Boolean)
        .join(" ");

      el.setAttribute(
        "aria-label",
        `Select ${asset.address}, ${asset.area} for assessment`
      );

      const scoreLabel = document.createElement("span");
      scoreLabel.textContent = markerLabel(asset, assessedScores);
      el.appendChild(scoreLabel);
      el.addEventListener("click", () => onSelectAsset(asset));

      return new maplibregl.Marker({ element: el, anchor: "center" })
        .setLngLat([asset.coordinates.lng, asset.coordinates.lat])
        .addTo(map);
    });

    return () => {
      markers.forEach((marker) => marker.remove());
    };
  }, [assets, selectedAsset?.id, onSelectAsset, assessedScores]);

  // Fly to selected asset when selection changes
  useEffect(() => {
    const map = mapRef.current;
    if (!map || !selectedAsset) return;

    map.flyTo({
      center: [selectedAsset.coordinates.lng, selectedAsset.coordinates.lat],
      zoom: Math.max(map.getZoom(), 12.6),
      speed: 0.85,
      curve: 1.25,
      essential: true,
    });
  }, [selectedAsset]);

  return (
    <div className="relative h-full min-h-0 bg-sunken">
      {/* Subtle analytical grid overlay */}
      <div
        aria-hidden="true"
        className="absolute inset-0 bg-[linear-gradient(rgba(34,211,238,0.04)_1px,transparent_1px),linear-gradient(90deg,rgba(34,211,238,0.04)_1px,transparent_1px)] bg-size-[64px_64px] pointer-events-none"
      />
      <div ref={containerRef} className="h-full w-full" />
      {/* Vignette: darkens edges to focus attention on center */}
      <div
        aria-hidden="true"
        className="pointer-events-none absolute inset-0 bg-[radial-gradient(circle_at_55%_45%,rgba(37,99,235,0.10),transparent_32%),linear-gradient(180deg,rgba(2,6,23,0.20),rgba(2,6,23,0.48))]"
      />
    </div>
  );
}
