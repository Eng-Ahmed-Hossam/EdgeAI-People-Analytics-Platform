import { cn } from "@/lib/utils";
import { HTMLAttributes, TdHTMLAttributes, ThHTMLAttributes } from "react";

export function Table({ className, children, ...props }: HTMLAttributes<HTMLTableElement>) {
  return (
    <div className="w-full overflow-x-auto">
      <table className={cn("w-full border-collapse text-sm", className)} {...props}>
        {children}
      </table>
    </div>
  );
}

export function Thead({ className, children, ...props }: HTMLAttributes<HTMLTableSectionElement>) {
  return (
    <thead className={cn("bg-sunken border-b border-hairline", className)} {...props}>
      {children}
    </thead>
  );
}

export function Tbody({ className, children, ...props }: HTMLAttributes<HTMLTableSectionElement>) {
  return (
    <tbody
      className={cn("divide-y divide-hairline", className)}
      {...props}
    >
      {children}
    </tbody>
  );
}

export function Th({ className, children, ...props }: ThHTMLAttributes<HTMLTableCellElement>) {
  return (
    <th
      className={cn(
        "px-4 py-2.5 text-left font-body text-xs font-medium uppercase tracking-widest text-ink-faint",
        className
      )}
      {...props}
    >
      {children}
    </th>
  );
}

export function Td({ className, children, ...props }: TdHTMLAttributes<HTMLTableCellElement>) {
  return (
    <td
      className={cn("px-4 py-3 font-body text-sm text-ink-body", className)}
      {...props}
    >
      {children}
    </td>
  );
}
