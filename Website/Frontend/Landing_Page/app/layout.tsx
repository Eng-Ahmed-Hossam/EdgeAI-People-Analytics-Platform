import '../styles/globals.css';

export const metadata = {
  title: 'FEASIX — Business Feasibility Powered by Location Intelligence',
  description:
    'Make confident business decisions using hardware-assisted location intelligence and financial feasibility analysis.',
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
