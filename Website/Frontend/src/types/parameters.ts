export type BusinessDomain =
  | "specialty_coffee"
  | "casual_dining"
  | "fast_food"
  | "retail_fashion"
  | "retail_grocery"
  | "pharmacy"
  | "gym_fitness"
  | "coworking"
  | "other";

export interface OperatingHours {
  open: string;  // "HH:mm"
  close: string;
  daysPerWeek: number;
}

export interface BusinessParameters {
  domain: BusinessDomain;
  operatingHours: OperatingHours;
  staffCount: number;
  budget: number;
  currency: string;
  additionalNotes?: string;
}
