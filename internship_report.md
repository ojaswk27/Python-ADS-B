# Summer Internship Report

**Student:** [Your Name]
**Roll No:** [Your Roll Number]
**Institution:** [Your Institution]
**Duration:** May 2026 to July 2026
**Company Guide:** [Name and Designation]

---

## 1. Name of the Organization and Department

The summer internship was completed at Bharat Electronics Limited, which is a Navratna public sector undertaking under the Ministry of Defence and one of India's leading defence electronics companies. BEL designs and manufactures a wide range of products including radars, communication systems, avionics, and surveillance equipment for the armed forces and also for civilian users.

Within BEL, the internship was carried out in the Development and Engineering department, which is usually called D&E internally. This is the group responsible for the design and development of new products and for the improvement of existing ones across BEL's various product lines. The work during the internship was aligned with the surveillance and Identification Friend or Foe systems team.

## 2. Roles and Responsibilities

The role assigned for the duration of the internship was that of a software development intern. Two main areas of work were given, and both continued through the eight weeks of the internship.

The first area was around evaluating newer programming tools and comparing them against the code that is already deployed in BEL's ADS-B units. The idea was to look at whether some of the newer options available today could offer improvements in areas like performance, maintainability, or general ease of working with the code, without breaking compatibility with the systems that are already in place. A lot of this work involved reading through existing code, understanding how it was structured, and running comparisons to see what actually held up when measured, as opposed to what only sounded promising on paper.

The second area, which took up the larger share of the time, was building software simulators for internal testing. The purpose of these simulators was to create controlled environments in which BEL's own surveillance software could be tested, without needing real hardware or expensive third party tools. Over the course of the internship, a fairly complete simulator was built that could generate aircraft data, decode it, run a simulated interrogating radar, and handle live inputs from real sources as well. The software also supported operator inputs through a graphical interface so that different test scenarios could be set up without anyone needing to change the code.

Alongside these two main threads of work, time was also spent attending team meetings, going through documentation and specifications provided by the company guide, and generally trying to understand how the various parts of a surveillance system fit together. Whenever specific questions came up, the company guide was very patient in explaining things and in providing useful guidance on where to look next.

## 3. Key Learnings

This internship offered a first proper exposure to how software is developed inside a large defence organisation, and a number of takeaways came out of it.

On the technical side, the internship offered hands on experience with concepts that had earlier only been familiar from reading, such as how aircraft position and identity data is transmitted, how radars perform interrogations, and how different systems communicate with each other through defined message formats over the network. Writing code against actual specification documents was quite different from following a textbook. Details that are easy to miss on a quick read of the spec often turned out to matter, and testing, rather than being something that could be tacked on at the end, worked much better when good test cases were written early.

On a slightly broader note, the importance of keeping code readable and well organised became evident, especially given that someone else may have to maintain the same code in the future. This point was made many times by the engineers on the team, and the reasoning behind it became clearer while going through code that had been written years earlier by different people. Taking notes properly during meetings and while reading documents was another useful habit picked up along the way, since a lot of the work depended on carrying context forward from one week to the next.

Working in a professional environment for the first time was itself a significant part of the learning. Adjusting to a slower and more careful pace than what is typical of college projects took some getting used to, in the sense that things had to be done with more thought given to the consequences of a change. It also became evident how the work of many different teams eventually comes together into a finished product, which was something not seen up close before.

## 4. Impact and Contribution

The contribution to the department has been in two forms, namely the simulator built during the internship and the observations from the tool evaluation work.

The simulator provides the team a way to test their software under controlled conditions without needing to bring in real hardware or wait for field trials, both of which are more expensive and slower. Since the simulator can be configured for different traffic scenarios and can accept inputs from external systems, it can also be used to reproduce situations that might otherwise be difficult to arrange. Parts of it are expected to be looked at for wider use within the team going forward, which is a positive outcome.

The benchmarking work has helped in a smaller but useful way, by giving the team some concrete measurements to base future decisions on. Instead of having to rely on general impressions about which tools are worth trying, there are now some numbers to refer to, and the notes and observations that have been handed over should be helpful when these decisions are actually made.

More generally, the work carried out during this internship is hoped to reflect well on the department at the parent institution. Sincere thanks are extended to BEL and to the company guide for the opportunity and support provided throughout the eight weeks.

---

*End of Report.*
