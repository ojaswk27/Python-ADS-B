# Summer Internship Learning Report

**Student:** [Your Name]
**Roll No / Registration No:** [Your Roll Number]
**Programme / Year:** [e.g. B.E. Electronics & Communication, Final Year]
**Institution:** [Your University / College]
**Submission Date:** [Date]

---

## 1. Name of the Organization and Department

**Organization:** Bharat Electronics Limited (BEL)
**Department:** Development and Engineering (D&E)

Bharat Electronics Limited is a Government of India Navratna public-sector
undertaking under the Ministry of Defence, engaged in the design and
manufacture of advanced electronic products for the Indian armed forces and
civilian users. The **Development and Engineering (D&E)** wing owns the
end-to-end product development lifecycle for BEL's electronic systems —
including the surveillance and Identification Friend-or-Foe (IFF) product
lines under which this internship was carried out.

---

## 2. Roles and Responsibilities

**Role:** Software Development Intern, D&E.

Over the course of the internship my responsibilities were:

1. **Tool benchmarking against deployed code.** Comparing newer programming
   tools, libraries, and language features against the currently-deployed
   codebases used in BEL's ADS-B (Automatic Dependent Surveillance –
   Broadcast) units, and assessing whether they were suitable for adoption
   in future revisions on grounds of performance, maintainability,
   correctness, and interoperability with existing byte-level protocols.

2. **Building simulators for internal testing.** Designing and implementing
   simulators that reproduce the ADS-B and IFF-radar operational
   environment in software. These simulators exercise the same on-the-wire
   message formats used by real hardware — enabling deployed and
   in-development BEL surveillance software to be tested without requiring
   real aircraft, real radar hardware, or expensive commercial simulators.

The concrete deliverables from these responsibilities included:

- A manual (pure-stdlib) ADS-B Mode S Extended Squitter decoder used as
  a byte-level reference against deployed decoders;
- An ADS-B emitter producing dump1090-compatible raw hex messages over
  UDP multicast;
- An interactive path-drawing emulator for authoring flight routes;
- An airspace simulator that owns ground truth and exposes it to a
  co-located software IFF radar scanner;
- The software IFF radar scanner itself, implementing a rotating antenna
  with configurable RPM, beamwidth, and PRT; all six interrogation modes
  (Mode 1, 2, 3/A, C, Mode S All-Call, Mode S Selective); binary reply
  framing; and remote mode control through a UDP interrogation message;
- Live ingestion of real ADS-B traffic into the same display for
  side-by-side visualisation and interrogation of simulated and observed
  aircraft.

The total delivery was approximately **5,400 lines of Python across
14 source modules**, verified end-to-end with a 42-test unit suite, several
byte-level format tests, and a set of scripted integration tests covering
external mode control, ADS-B ingestion, and beam-sweep coverage.

---

## 3. Key Learnings

*TODO — fill in.*

*(Personal reflection on what was learned during the internship: technical
skills, tools, engineering practices, exposure to the defence electronics
domain, and any takeaways from working within a large PSU R&D group.)*

---

## 4. Impact and Contribution

*TODO — fill in.*

*(Description of how the delivered work benefits the department: e.g. how
the simulator supports test-driven development of deployed code, how the
tool-benchmarking analysis informed future adoption decisions, whether any
of the code has been picked up for further internal use, and any measurable
outcomes.)*

---

*End of Report.*
