import { clsx, type ClassValue } from "clsx";
import { twMerge } from "tailwind-merge";
import type { ScoreBand } from "@/types/report";

export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs));
}

export function formatCurrency(
  amount: number,
  currency: string = "EGP"
): string {
  return new Intl.NumberFormat("en-EG", {
    style: "currency",
    currency,
    maximumFractionDigits: 0,
  }).format(amount);
}

export function formatNumber(n: number): string {
  return new Intl.NumberFormat("en-EG").format(n);
}

export function formatDate(iso: string): string {
  return new Intl.DateTimeFormat("en-GB", {
    day: "numeric",
    month: "short",
    year: "numeric",
    hour: "2-digit",
    minute: "2-digit",
  }).format(new Date(iso));
}

export function bandColors(band: ScoreBand) {
  switch (band) {
    case "strong":
      return {
        text: "text-verdict-strong",
        bg: "bg-verdict-strong-wash",
        border: "border-verdict-good",
      };
    case "moderate":
      return {
        text: "text-verdict-warn",
        bg: "bg-verdict-warn-wash",
        border: "border-verdict-warn",
      };
    case "weak":
      return {
        text: "text-verdict-weak",
        bg: "bg-verdict-weak-wash",
        border: "border-verdict-weak",
      };
  }
}

export function bandLabel(band: ScoreBand): string {
  switch (band) {
    case "strong":
      return "Strong Feasibility";
    case "moderate":
      return "Moderate Feasibility";
    case "weak":
      return "Weak Feasibility";
  }
}
