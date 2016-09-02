using System;
using System.Collections.Generic;
using Newtonsoft.Json;
using Newtonsoft.Json.Serialization;
using System.Linq;

namespace PersonalRobotics.Kinect2Server
{
    public class BodyContractResolver : DefaultContractResolver
    {
        protected override IList<JsonProperty> CreateProperties(Type type, MemberSerialization memberSerialization)
        {
            IList<JsonProperty> properties = base.CreateProperties(type, memberSerialization);

            properties = properties
                .Where(p => !p.PropertyName.Equals("Activities"))
                .Where(p => !p.PropertyName.Equals("Appearance"))
                .Where(p => !p.PropertyName.Equals("Engaged"))
                .Where(p => !p.PropertyName.Equals("Expressions"))
                .Where(p => !p.PropertyName.Equals("IsTracked"))
                .ToList();

            return properties;
        }
    }

    public class AudioContractResolver : DefaultContractResolver
    {
        protected override IList<JsonProperty> CreateProperties(Type type, MemberSerialization memberSerialization)
        {
            IList<JsonProperty> properties = base.CreateProperties(type, memberSerialization);
            return properties;
        }
    }

    public class FaceContractResolver : DefaultContractResolver
    {
        protected override IList<JsonProperty> CreateProperties(Type type, MemberSerialization memberSerialization)
        {
            IList<JsonProperty> properties = base.CreateProperties(type, memberSerialization);

            properties = properties
                .Where(p => !p.PropertyName.Equals("Quality"))
                .ToList();

            return properties;
        }
    }
}
