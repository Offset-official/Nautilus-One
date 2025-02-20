"use client";
import Link from "next/link";

export default function Navbar() {
    return (
        <nav className="w-full h-16 bg-gray-800 flex items-center px-6 shadow-md fixed top-0 left-0">
            <h1 className="text-white text-xl font-bold">Nautilus One</h1>
            <div className="ml-auto">
                <Link href="" className="text-white">Home</Link>
                <Link href="streams" className="text-white ml-4">Streams</Link>
            </div>
        </nav>
    );
}
